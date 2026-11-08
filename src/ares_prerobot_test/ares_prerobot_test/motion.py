"""Jerk-limited tracking of planar and angular velocity targets."""
import math


def limit(values, maximum):
    size = math.hypot(*values)
    if size <= maximum:
        return values
    scale = maximum / size
    return [value * scale for value in values]


class Motion:
    def __init__(self, angular_accel=2.0, angular_jerk=8.0):
        self.v = [0.0, 0.0, 0.0]
        self.a = [0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, 0.0]
        self.amax = 1.0
        self.jerkmax = 2.0
        self.angular_accel = angular_accel
        self.angular_jerk = angular_jerk

    def set_target(self, vx, vy, wz, amax, jerkmax):
        self.target = [vx, vy, wz]
        self.amax = amax
        self.jerkmax = jerkmax

    def halt(self):
        self.target = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.a = [0.0, 0.0, 0.0]

    def step(self, dt):
        error = [self.target[i] - self.v[i] for i in range(2)]
        desired = limit([8.0 * error[i] - 4.0 * self.a[i] for i in range(2)], self.amax)
        change = limit([desired[i] - self.a[i] for i in range(2)], self.jerkmax * dt)
        for i in range(2):
            self.a[i] += change[i]
            self.v[i] += self.a[i] * dt
        desired_w = max(-self.angular_accel, min(self.angular_accel,
            8.0 * (self.target[2] - self.v[2]) - 4.0 * self.a[2]))
        delta_w = max(-self.angular_jerk * dt, min(self.angular_jerk * dt,
            desired_w - self.a[2]))
        self.a[2] += delta_w
        self.v[2] += self.a[2] * dt
        if math.hypot(self.target[0] - self.v[0], self.target[1] - self.v[1]) < 0.001 \
                and math.hypot(self.a[0], self.a[1]) <= self.jerkmax * dt:
            self.v[:2] = self.target[:2]
            self.a[:2] = [0.0, 0.0]
        if abs(self.target[2] - self.v[2]) < 0.001 \
                and abs(self.a[2]) <= self.angular_jerk * dt:
            self.v[2] = self.target[2]
            self.a[2] = 0.0
        return self.v[:]

    def at_target(self):
        return self.v == self.target and self.a == [0.0, 0.0, 0.0]
