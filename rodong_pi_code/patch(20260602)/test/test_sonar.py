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
    assert front_min(s) == 30          # ignore -1, min(50,30)
    assert front_min([0] * 8) == 999   # all invalid → 999


def test_rear_min():
    s = _sonar(**{'5': 40, '6': 0, '7': 25})
    assert rear_min(s) == 25


def test_build_histogram_marks_near_front_blocked():
    # front beam (idx3, 0°, sector3) at 10cm → weight=(40-10)/40=0.75
    s = _sonar(**{'3': 10})
    hist = build_histogram(s)
    assert hist[3] > cfg.OPEN_THRESH                  # sector3 blocked
    assert abs(hist[2] - 0.375) < 1e-6                # adjacent bleed 0.75*0.5
    assert abs(hist[4] - 0.375) < 1e-6
    assert hist[0] == 0.0                             # side/rear beams have no effect


def test_build_histogram_ignores_far_and_rear():
    # all far (>=40) or rear beams → all 0
    s = _sonar(**{'6': 5, '7': 5, '5': 5})            # only rear is close
    hist = build_histogram(s)
    assert hist == [0.0] * cfg.N_SECTORS


def test_select_sector_picks_goal_when_open():
    hist = [0.0] * 7
    assert select_sector(hist, goal_sector=6, prev_steer=0) == 6
    assert select_sector(hist, goal_sector=3, prev_steer=0) == 3


def test_select_sector_avoids_blocked_front():
    # front (sector3) blocked → to an adjacent passable sector (2 or 4)
    s = _sonar(**{'3': 10})
    hist = build_histogram(s)
    sec = select_sector(hist, goal_sector=3, prev_steer=0)
    assert sec in (2, 4)


def test_select_sector_all_blocked_returns_none():
    hist = [1.0] * 7
    assert select_sector(hist, goal_sector=3, prev_steer=0) is None
