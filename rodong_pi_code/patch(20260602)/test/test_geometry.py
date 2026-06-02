# -*- coding: utf-8 -*-
from rodong_geometry import yaw_diff, yaw_signed_diff, angle_to_sector, clip


def test_clip():
    assert clip(5, 0, 10) == 5
    assert clip(-3, 0, 10) == 0
    assert clip(99, 0, 10) == 10


def test_yaw_diff_basic():
    assert yaw_diff(0, 0) == 0
    assert yaw_diff(0, 90) == 90
    assert yaw_diff(0, 180) == 180


def test_yaw_diff_wraparound():
    # 350° 와 10° 는 20° 차이 (경계 넘김)
    assert yaw_diff(350, 10) == 20
    assert yaw_diff(10, 350) == 20
    # 항상 0~180 범위
    assert yaw_diff(0, 270) == 90


def test_yaw_signed_diff_sign():
    # target 이 current 보다 크면 + (좌회전 필요)
    assert yaw_signed_diff(10, 0) == 10
    assert yaw_signed_diff(0, 10) == -10
    # 경계 정규화: -170 - 170 = -340 → +20
    assert yaw_signed_diff(-170, 170) == 20
    assert yaw_signed_diff(170, -170) == -20


def test_angle_to_sector_centers():
    assert angle_to_sector(-90) == 0
    assert angle_to_sector(0) == 3
    assert angle_to_sector(90) == 6
    assert angle_to_sector(30) == 4


def test_angle_to_sector_out_of_range():
    assert angle_to_sector(180) is None
    assert angle_to_sector(-135) is None
    # 경계 안쪽은 클램프되어 유효
    assert angle_to_sector(105) == 6
    assert angle_to_sector(-105) == 0
