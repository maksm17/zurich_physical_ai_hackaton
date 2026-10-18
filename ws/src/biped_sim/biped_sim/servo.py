"""ST3215 behaviour on top of MuJoCo's position actuator, fitted to the real servo (l_down test):
a commanded target reaches the servo after a dead time (ROS topic + controller + UART + servo
firmware), then the servo's goal follows it with a short first-order lag, and errors below the
deadband are not corrected.

Real measurement (scripts/servo_test.py): 3 deg step starts moving after ~36 ms (raw UART),
90 % after ~70 ms; through ros2_control the sine 0.5 Hz +-5 deg lags ~80 ms at 88 % amplitude;
steady error up to ~0.35 deg.
"""
import collections

import numpy as np

REAL = dict(delay=0.035, tau=0.015, deadband=0.008)


class ServoModel:
    """Call update(target, q, dt) every physics step; returns the goal to give the actuator."""

    def __init__(self, n, delay=REAL['delay'], tau=REAL['tau'], deadband=REAL['deadband'],
                 dt=0.002):
        self.dt = dt
        self.set(delay, tau, deadband)
        self.reset(np.zeros(n))

    def set(self, delay, tau, deadband):
        self.delay, self.tau, self.deadband = delay, tau, deadband
        self.n_delay = max(0, int(round(delay / self.dt)))

    def reset(self, q):
        q = np.asarray(q, dtype=float)
        self.queue = collections.deque([q.copy()] * (self.n_delay + 1), maxlen=self.n_delay + 1)
        self.goal = q.copy()
        self.hold = q.copy()

    def update(self, target, q):
        self.queue.append(np.asarray(target, dtype=float).copy())
        delayed = self.queue[0]
        a = min(1.0, self.dt / self.tau) if self.tau > 0 else 1.0
        self.goal += a * (delayed - self.goal)
        far = np.abs(self.goal - q) > self.deadband
        self.hold = np.where(far, self.goal, self.hold)
        return self.hold
