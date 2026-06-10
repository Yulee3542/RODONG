# -*- coding: utf-8 -*-
"""
rodong_control.py — controllers (pure, no ROS dependency)
================================================================
Simple PID, used for closed-loop control such as marker approach
(bearing→steering) and heading recovery (yaw error→steering). Unit-testable.
"""


class PID:
    """Standard PID with output/integral clamping.

    The output is sign-neutral (out = kp*e + ki*∫e + kd*de/dt); the steering sign
    convention (e.g. err>0 → negative steering for heading recovery) is handled by
    the caller.
    """

    def __init__(self, kp, ki=0.0, kd=0.0, out_limit=None, i_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_err = None

    def step(self, error, dt):
        if dt <= 0.0:
            dt = 1e-3
        self._integral += error * dt
        if self.i_limit is not None:
            self._integral = max(-self.i_limit, min(self.i_limit, self._integral))

        deriv = 0.0 if self._prev_err is None else (error - self._prev_err) / dt
        self._prev_err = error

        out = self.kp * error + self.ki * self._integral + self.kd * deriv
        if self.out_limit is not None:
            out = max(-self.out_limit, min(self.out_limit, out))
        return out
