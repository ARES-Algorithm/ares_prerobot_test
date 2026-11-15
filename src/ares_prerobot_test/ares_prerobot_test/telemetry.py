"""Bounded live wheel telemetry; never generates motion commands."""
from collections import deque
import math
import threading
import time

import numpy as np


class Derivatives:
    """Causal quadratic fit on real timestamps; estimate at window centre.

    An acceleration/jerk point has half-window latency, not an invented sample
    at the current instant. Missing data clears the fit instead of bridging it.
    """

    def __init__(self, window=.24, max_gap=.15):
        self.window, self.max_gap = window, max_gap
        self.points = deque()

    def add(self, stamp, value):
        if not math.isfinite(stamp) or not math.isfinite(value):
            self.points.clear()
            return None
        if self.points:
            dt = stamp - self.points[-1][0]
            if dt <= 0:
                return None
            if dt > self.max_gap:
                self.points.clear()
        self.points.append((stamp, value))
        while self.points and stamp - self.points[0][0] > self.window + 1e-8:
            self.points.popleft()
        if len(self.points) < 7 or stamp - self.points[0][0] < self.window * .8:
            return None
        centre = (self.points[0][0] + stamp) / 2
        x = np.array([p[0] - centre for p in self.points])
        y = np.array([p[1] for p in self.points])
        c = np.polynomial.polynomial.polyfit(x, y, 2)
        return dict(t=centre, v=float(c[0]), a=float(c[1]), j=float(2*c[2]))


class LiveTelemetry:
    def __init__(self, radius=.04, rpm_ratio=1., window=.24, history=30., clock=time.monotonic):
        if any(not math.isfinite(x) or x <= 0 for x in (radius, rpm_ratio, window, history)):
            raise ValueError('radius, rpm_ratio, window and history must be positive')
        self.lock = threading.RLock()
        self.clock = clock
        self.radius, self.rpm_ratio, self.window, self.history = radius, rpm_ratio, window, history
        self.fit = [Derivatives(window) for _ in range(4)]
        self.command_fit = Derivatives(window)
        self.wheels = [deque(maxlen=5000) for _ in range(4)]
        self.currents = deque(maxlen=5000)
        self.commands = deque(maxlen=10000)
        self.last_sensor = None
        self.last_command = None
        self.last_sequence = None
        self.last_uptime = None
        self.sequence = 0
        self.epoch = 0
        self.sensor_origin = None
        self.error = None

    @staticmethod
    def finite(values):
        return all(math.isfinite(float(x)) for x in values)

    def command(self, vx, vy, received=None):
        if not self.finite((vx, vy)):
            return
        now = self.clock() if received is None else received
        with self.lock:
            if self.last_command is not None and now <= self.last_command:
                return
            speed = math.hypot(vx, vy)
            fit = self.command_fit.add(now, speed)
            self.sequence += 1
            self.commands.append(dict(id=self.sequence, received=now, t=now, v=speed,
                                      derived=fit))
            self.last_command = now

    def raw(self, data_id, values):
        now = self.clock()
        with self.lock:
            if data_id == 0x0131:
                # chassis_speed_test: seq, uptime(sec/usec), 4 drive RPM, 4 steering RPM.
                if len(values) != 11:
                    self.error = '转速遥测格式不匹配：需要 chassis_speed_test 的 11 float 包'
                    return
                if not self.finite(values[:3]):
                    return
                seq, sec, usec = values[:3]
                if seq != int(seq) or not 0 <= seq < 2**24 or sec < 0 or sec != int(sec) or not 0 <= usec < 1e6:
                    return
                uptime = sec + usec * 1e-6
                # A restart establishes a new epoch; out-of-order/repeated packets are ignored.
                if self.last_uptime is not None and uptime <= self.last_uptime:
                    if uptime < self.last_uptime - 1. and seq < 10:
                        self.reset_sensor()
                    else:
                        return
                if self.last_sequence == seq:
                    return
                if self.sensor_origin is None:
                    self.sensor_origin = now - uptime
                stamp = self.sensor_origin + uptime
                # Re-anchor after a long disconnect; do not plot stale data as fresh.
                if abs(stamp - now) > .5:
                    self.reset_sensor()
                    self.sensor_origin = now - uptime
                    stamp = now
                self.last_uptime, self.last_sequence = uptime, seq
                self.last_sensor = now
                self.error = None
                for i, rpm in enumerate(values[3:7]):
                    self.sequence += 1
                    # Convert motor RPM to wheel-rim speed; the sign is irrelevant to speed magnitude.
                    v = abs(float(rpm)) * 2 * math.pi * self.radius / (60 * self.rpm_ratio)
                    derived = self.fit[i].add(stamp, v)
                    self.wheels[i].append(dict(id=self.sequence, received=now, t=stamp,
                                               seq=int(seq), rpm=float(rpm) if math.isfinite(rpm) else None,
                                               v=v if math.isfinite(v) else None, derived=derived))
            elif data_id == 0x0141:
                # seq, uptime seconds, 4 drive currents (Id), 4 steering currents (Is).
                if len(values) != 10 or not self.finite(values[:2]):
                    return
                seq = values[0]
                if seq != int(seq) or not 0 <= seq < 2**24:
                    return
                if self.currents and self.currents[-1]['seq'] == int(seq):
                    return
                self.sequence += 1
                self.currents.append(dict(id=self.sequence, received=now, t=now, seq=int(seq),
                    Id=[float(v) if math.isfinite(v) else None for v in values[2:6]],
                    Is=[float(v) if math.isfinite(v) else None for v in values[6:10]]))
            self.trim(now)

    def reset_sensor(self):
        for rows, fit in zip(self.wheels, self.fit):
            rows.clear()
            fit.points.clear()
        self.currents.clear()
        self.last_uptime = self.last_sequence = self.sensor_origin = None
        self.epoch += 1

    def trim(self, now):
        for queue in [*self.wheels, self.currents, self.commands]:
            while queue and now - queue[0]['received'] > self.history:
                queue.popleft()

    def snapshot(self, wheel=0, since=0):
        if wheel not in range(4):
            raise ValueError('wheel must be 0..3')
        with self.lock:
            now = self.clock()
            self.trim(now)
            wheel_rows = list(self.wheels[wheel])
            # Both packets share a sample sequence; use the RPM timestamp for aligned plots.
            times = {r['seq']: r['t'] for r in wheel_rows}
            currents = [dict(id=r['id'], t=times[r['seq']], Is=r['Is'][wheel], Id=r['Id'][wheel])
                        for r in self.currents if r['id'] > since and r['seq'] in times]
            return dict(now=now, epoch=self.epoch, cursor=self.sequence,
                        sensor_age=None if self.last_sensor is None else now-self.last_sensor,
                        command_age=None if self.last_command is None else now-self.last_command,
                        error=self.error, radius=self.radius, rpm_ratio=self.rpm_ratio,
                        derivative_window=self.window, history=self.history,
                        wheel=[r for r in wheel_rows if r['id'] > since], current=currents,
                        command=[r for r in self.commands if r['id'] > since])
