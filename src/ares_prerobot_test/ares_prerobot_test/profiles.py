"""Deterministic velocity excitation in SI units."""
import math

class Profile:
    def __init__(self, kind, speed, acceleration, jerk, hold):
        if kind not in ('speed', 'acceleration', 'jerk'):
            raise ValueError('kind must be speed, acceleration or jerk')
        if any(not math.isfinite(x) or x <= 0 for x in (speed, acceleration, jerk, hold)):
            raise ValueError('speed, acceleration, jerk and hold must be finite and positive')
        self.kind, self.v, self.a, self.j, self.hold = kind, speed, acceleration, jerk, hold
        if kind == 'jerk':
            self.tj = min(acceleration / jerk, math.sqrt(speed / jerk))
            self.ta = max(0., speed / (jerk * self.tj) - self.tj)
            self.ramp = 2*self.tj+self.ta
        else:
            self.ramp = speed / acceleration
        self.duration = 2*self.ramp + hold

    def up(self, t):
        t = min(max(t, 0.), self.ramp)
        if self.kind != 'jerk':
            return min(self.v, self.a*t)
        tj, ta, j = self.tj, self.ta, self.j
        if t < tj:
            return .5*j*t*t
        if t < tj+ta:
            return .5*j*tj*tj+j*tj*(t-tj)
        return self.v-.5*j*(self.ramp-t)**2

    def sample(self, t):
        if t < 0 or t >= self.duration:
            return 0., 'finished' if t >= self.duration else 'prepare'
        if t < self.ramp:
            # Acceleration test requests a step; actual firmware ramp is recorded separately.
            return (self.v if self.kind == 'acceleration' else self.up(t)), 'accelerate'
        if t < self.ramp+self.hold:
            return self.v, 'hold'
        return self.v-self.up(t-self.ramp-self.hold), 'decelerate'
