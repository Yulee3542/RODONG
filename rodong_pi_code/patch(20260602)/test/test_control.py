# -*- coding: utf-8 -*-
from rodong_control import PID


def test_pure_p():
    pid = PID(kp=2.0)
    assert pid.step(3.0, dt=0.1) == 6.0      # 2*3
    assert pid.step(-1.0, dt=0.1) == -2.0


def test_derivative_term():
    pid = PID(kp=0.0, kd=1.0)
    # first step has no prev_err → derivative 0
    assert pid.step(1.0, dt=1.0) == 0.0
    # error 1→3, dt=1 → de/dt=2 → output 2
    assert pid.step(3.0, dt=1.0) == 2.0


def test_integral_accumulates():
    pid = PID(kp=0.0, ki=1.0)
    assert abs(pid.step(2.0, dt=0.5) - 1.0) < 1e-9   # ∫=2*0.5=1
    assert abs(pid.step(2.0, dt=0.5) - 2.0) < 1e-9   # ∫=2.0


def test_output_clamp():
    pid = PID(kp=100.0, out_limit=90.0)
    assert pid.step(10.0, dt=0.1) == 90.0
    assert pid.step(-10.0, dt=0.1) == -90.0


def test_integral_clamp():
    pid = PID(kp=0.0, ki=1.0, i_limit=1.0)
    pid.step(10.0, dt=1.0)                    # ∫ would be 10 → clamp 1
    assert abs(pid._integral - 1.0) < 1e-9


def test_reset_clears_state():
    pid = PID(kp=0.0, kd=1.0)
    pid.step(5.0, dt=1.0)
    pid.reset()
    # after reset, the first step is derivative 0 again
    assert pid.step(9.0, dt=1.0) == 0.0
