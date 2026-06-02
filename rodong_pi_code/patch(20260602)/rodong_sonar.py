# -*- coding: utf-8 -*-
"""
rodong_sonar.py — 초음파 융합 / VFH 히스토그램 (순수 함수, ROS 의존성 없음)
================================================================
vfh_planner.cb_timer 와 rodong_main 의 초음파 처리 로직을 추출.
모두 평범한 list 를 받으므로 ROS 없이 단위테스트 가능. 동작은 기존과 동일하다.
"""

import rodong_config as cfg
from rodong_geometry import angle_to_sector


def front_min(sonar):
    """전방 3빔(좌전/우전/정면) 중 유효 최소거리 [cm]. 없으면 999."""
    vals = [sonar[i] for i in cfg.FRONT_IDXS if sonar[i] > 0]
    return min(vals) if vals else 999


def rear_min(sonar):
    """후방 3빔(우후/후/좌후) 중 유효 최소거리 [cm]. 없으면 999."""
    vals = [sonar[i] for i in cfg.REAR_IDXS if sonar[i] > 0]
    return min(vals) if vals else 999


def build_histogram(sonar, beam_angles=None, threshold=None,
                    n_sectors=None, sector_deg=None, climb_now=False):
    """초음파 8빔 → 전방 7섹터 polar histogram (장애물 밀집도).

    climb_now=True 면 정면(±45°) 장애물 영향을 완화(넘어갈 대상)한다.
    """
    if beam_angles is None:
        beam_angles = cfg.BEAM_ANGLES
    if threshold is None:
        threshold = cfg.THRESHOLD
    if n_sectors is None:
        n_sectors = cfg.N_SECTORS
    if sector_deg is None:
        sector_deg = cfg.SECTOR_DEG

    hist = [0.0] * n_sectors
    for i, ang in enumerate(beam_angles):
        s = angle_to_sector(ang, n_sectors, sector_deg)
        if s is None:
            continue                          # 후방 빔 무시
        dist = sonar[i]
        if dist <= 0:
            continue
        if dist < threshold:
            if climb_now and abs(ang) <= 45:
                continue
            weight = (threshold - dist) / threshold
            hist[s] += weight
            if s > 0:
                hist[s - 1] += weight * 0.5   # 인접 섹터 번짐
            if s < n_sectors - 1:
                hist[s + 1] += weight * 0.5
    return hist


def select_sector(hist, goal_sector, prev_steer,
                  sector_angle=None, open_thresh=None,
                  w_goal=None, w_heading=None, w_smooth=None):
    """통행 가능 섹터 중 비용 최소 섹터 인덱스. 전부 막히면 None."""
    if sector_angle is None:
        sector_angle = cfg.SECTOR_ANGLE
    if open_thresh is None:
        open_thresh = cfg.OPEN_THRESH
    if w_goal is None:
        w_goal = cfg.W_GOAL
    if w_heading is None:
        w_heading = cfg.W_HEADING
    if w_smooth is None:
        w_smooth = cfg.W_SMOOTH

    best_sector, best_cost = None, float('inf')
    for s in range(len(hist)):
        if hist[s] > open_thresh:
            continue
        diff_goal    = abs(s - goal_sector)
        diff_heading = abs(sector_angle[s] - 0)            # 직진 대비
        diff_smooth  = abs(sector_angle[s] - prev_steer)
        cost = (w_goal * diff_goal +
                w_heading * (diff_heading / 30.0) +
                w_smooth * (diff_smooth / 30.0))
        if cost < best_cost:
            best_cost, best_sector = cost, s
    return best_sector
