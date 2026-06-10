# -*- coding: utf-8 -*-
"""
rodong_sonar.py — sonar fusion / VFH histogram (pure functions, no ROS dependency)
================================================================
The ultrasonic-processing logic of vfh_planner.cb_timer and rodong_main is
extracted here. Everything takes plain lists, so it is unit-testable without ROS.
Behavior is identical to before.
"""

import rodong_config as cfg
from rodong_geometry import angle_to_sector


def front_min(sonar):
    """Min valid distance [cm] among the 3 front beams (front-left/front-right/front). 999 if none."""
    vals = [sonar[i] for i in cfg.FRONT_IDXS if sonar[i] > 0]
    return min(vals) if vals else 999


def rear_min(sonar):
    """Min valid distance [cm] among the 3 rear beams (rear-right/rear/rear-left). 999 if none."""
    vals = [sonar[i] for i in cfg.REAR_IDXS if sonar[i] > 0]
    return min(vals) if vals else 999


def build_histogram(sonar, beam_angles=None, threshold=None,
                    n_sectors=None, sector_deg=None, climb_now=False):
    """8 sonar beams → front 7-sector polar histogram (obstacle density).

    If climb_now=True, the influence of front (±45°) obstacles is relaxed
    (they are objects to climb over).
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
            continue                          # ignore rear beams
        dist = sonar[i]
        if dist <= 0:
            continue
        if dist < threshold:
            if climb_now and abs(ang) <= 45:
                continue
            weight = (threshold - dist) / threshold
            hist[s] += weight
            if s > 0:
                hist[s - 1] += weight * 0.5   # bleed into adjacent sector
            if s < n_sectors - 1:
                hist[s + 1] += weight * 0.5
    return hist


def select_sector(hist, goal_sector, prev_steer,
                  sector_angle=None, open_thresh=None,
                  w_goal=None, w_heading=None, w_smooth=None):
    """Index of the min-cost passable sector. None if everything is blocked."""
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
        diff_heading = abs(sector_angle[s] - 0)            # relative to straight
        diff_smooth  = abs(sector_angle[s] - prev_steer)
        cost = (w_goal * diff_goal +
                w_heading * (diff_heading / 30.0) +
                w_smooth * (diff_smooth / 30.0))
        if cost < best_cost:
            best_cost, best_sector = cost, s
    return best_sector
