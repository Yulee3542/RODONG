# -*- coding: utf-8 -*-
"""
rodong_geometry.py — 순수 기하/각도 헬퍼 (ROS 의존성 없음)
================================================================
rodong_main / vfh_planner 에 흩어져 있던 각도 계산을 추출하여 단위테스트 가능하게.
동작은 기존과 동일하다.
"""


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_diff(start, current):
    """두 yaw[deg] 사이의 절대 차이 (0~180)."""
    d = abs(current - start) % 360
    return d if d <= 180 else 360 - d


def yaw_signed_diff(target, current):
    """target - current 를 (-180, 180]° 로 정규화.
    부호: + → yaw 증가 필요(좌회전), - → yaw 감소 필요(우회전)."""
    return (target - current + 180) % 360 - 180


def angle_to_sector(deg, n_sectors=7, sector_deg=30.0):
    """-90~+90 deg → 0~(n-1) 섹터. 범위(±105) 밖이면 None."""
    if deg < -105 or deg > 105:
        return None
    s = int(round((deg + 90) / sector_deg))
    return max(0, min(n_sectors - 1, s))
