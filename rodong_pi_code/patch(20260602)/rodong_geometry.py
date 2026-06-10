# -*- coding: utf-8 -*-
"""
rodong_geometry.py — pure geometry/angle helpers (no ROS dependency)
================================================================
Angle math that used to be scattered across rodong_main / vfh_planner, extracted
so it can be unit-tested. Behavior is identical to before.
"""


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_diff(start, current):
    """Absolute difference between two yaw[deg] values (0..180)."""
    d = abs(current - start) % 360
    return d if d <= 180 else 360 - d


def yaw_signed_diff(target, current):
    """Normalize (target - current) to (-180, 180] degrees.
    Sign: + → yaw must increase (turn left), - → yaw must decrease (turn right)."""
    return (target - current + 180) % 360 - 180


def angle_to_sector(deg, n_sectors=7, sector_deg=30.0):
    """-90..+90 deg → sector 0..(n-1). Returns None if outside the range (±105)."""
    if deg < -105 or deg > 105:
        return None
    s = int(round((deg + 90) / sector_deg))
    return max(0, min(n_sectors - 1, s))
