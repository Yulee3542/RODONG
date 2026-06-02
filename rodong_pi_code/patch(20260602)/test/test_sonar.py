# -*- coding: utf-8 -*-
import rodong_config as cfg
from rodong_sonar import front_min, rear_min, build_histogram, select_sector

FAR = 999


def _sonar(**idx_val):
    s = [FAR] * 8
    for i, v in idx_val.items():
        s[int(i)] = v
    return s


def test_front_min_ignores_nonpositive():
    s = _sonar(**{'1': 50, '2': -1, '3': 30})
    assert front_min(s) == 30          # -1 무시, min(50,30)
    assert front_min([0] * 8) == 999   # 전부 무효 → 999


def test_rear_min():
    s = _sonar(**{'5': 40, '6': 0, '7': 25})
    assert rear_min(s) == 25


def test_build_histogram_marks_near_front_blocked():
    # 정면 빔(idx3, 0°, 섹터3)이 10cm → weight=(40-10)/40=0.75
    s = _sonar(**{'3': 10})
    hist = build_histogram(s)
    assert hist[3] > cfg.OPEN_THRESH                  # 섹터3 막힘
    assert abs(hist[2] - 0.375) < 1e-6                # 인접 번짐 0.75*0.5
    assert abs(hist[4] - 0.375) < 1e-6
    assert hist[0] == 0.0                             # 측/후방 빔 영향 없음


def test_build_histogram_ignores_far_and_rear():
    # 모두 멀거나(>=40) 후방 빔 → 전부 0
    s = _sonar(**{'6': 5, '7': 5, '5': 5})            # 후방만 가까움
    hist = build_histogram(s)
    assert hist == [0.0] * cfg.N_SECTORS


def test_select_sector_picks_goal_when_open():
    hist = [0.0] * 7
    assert select_sector(hist, goal_sector=6, prev_steer=0) == 6
    assert select_sector(hist, goal_sector=3, prev_steer=0) == 3


def test_select_sector_avoids_blocked_front():
    # 정면(섹터3) 막힘 → 인접 통행 섹터(2 또는 4)로
    s = _sonar(**{'3': 10})
    hist = build_histogram(s)
    sec = select_sector(hist, goal_sector=3, prev_steer=0)
    assert sec in (2, 4)


def test_select_sector_all_blocked_returns_none():
    hist = [1.0] * 7
    assert select_sector(hist, goal_sector=3, prev_steer=0) is None
