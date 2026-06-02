# -*- coding: utf-8 -*-
"""
rodong_control.py — 제어기 (순수, ROS 의존성 없음)
================================================================
간단한 PID. 마커 접근(bearing→조향), 헤딩 복귀(yaw오차→조향) 등
폐루프 제어에 사용. 단위테스트 가능.
"""


class PID:
    """표준 PID. 출력/적분 클램프 지원.

    출력은 부호 중립(out = kp*e + ki*∫e + kd*de/dt)이며,
    조향 부호 규약(예: 헤딩 복귀의 err>0 → 음의 조향)은 호출부에서 처리한다.
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
