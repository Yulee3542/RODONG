#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
uturn_block.py
패치 스크립트가 실패할 경우, 이 파일의 내용을 rodong_main.py에 수동으로 복붙한다.

[1] 상수 섹션 교체 (기존 UTURN 상수들 자리에)
[2] 메서드 섹션 교체 (yaw_diff + execute_uturn + 헬퍼들)
"""

# ═══════════════════════════════════════════════════════════════════════════
# [1] 상수 섹션  ─  기존 UTURN 상수 블록 전체를 이것으로 교체
# ═══════════════════════════════════════════════════════════════════════════
"""
# ── UTURN 파라미터 ────────────────────────────────────────────────────────
# IMU가 /imu/data 를 발행하면 자동으로 yaw 기반으로 동작,
# 없으면 UTURN_STEPS 시간 기반으로 fallback.
UTURN_TARGET_DEG  = 175.0   # 목표 누적 회전량 (°)
UTURN_PHASE1_TO   = 5.0     # Phase1 독립 타임아웃 (s)
UTURN_PHASE2_TO   = 5.0     # Phase2 독립 타임아웃 (s)

# 시간 기반 fallback: (speed, angle, duration_s)
UTURN_STEPS = [
    ( 20,  90, 1.4),   # 전진 우회전
    (-20, -90, 1.4),   # 후진 좌회전
    ( 20,  90, 1.4),   # 전진 우회전
    (-20, -90, 1.4),   # 후진 좌회전
]
"""

# ═══════════════════════════════════════════════════════════════════════════
# [2] 메서드 섹션  ─  RodongMain 클래스 안, 기존 yaw_diff + execute_uturn 자리에
# ═══════════════════════════════════════════════════════════════════════════
"""
    def yaw_diff(self, start, current):
        d = abs(current - start) % 360
        return d if d <= 180 else 360 - d

    # ══════════════════════════════════════════════════════════════════════
    # UTURN — 옵션C 하이브리드
    #   IMU 사용 가능  →  yaw 기반 Phase1(전진+우90°) + Phase2(후진+좌 나머지)
    #   IMU 없음       →  UTURN_STEPS 시간 기반 fallback
    # ══════════════════════════════════════════════════════════════════════
    def execute_uturn(self):
        if self.imu_ready:
            self._uturn_imu()
        else:
            rospy.logwarn('[UTURN] IMU 없음 → 시간 기반 fallback')
            self._uturn_timed()
        rospy.loginfo('[UTURN] 완료 → BUG_DRIVE')
        self.state = State.BUG_DRIVE

    def _uturn_imu(self):
        rospy.loginfo('[UTURN/IMU] 시작')
        rate = rospy.Rate(20)

        # Phase 1: 전진 + 우회전 → UTURN_TARGET_DEG/2 목표
        rospy.loginfo('[UTURN/IMU] Phase1: 전진+우90°')
        p1_start  = self.yaw_deg
        p1_target = UTURN_TARGET_DEG / 2.0
        t1 = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t1).to_sec()
            turned  = self.yaw_diff(p1_start, self.yaw_deg)
            rospy.loginfo_throttle(0.5, '[UTURN/IMU] P1 %.1f/%.1f° (%.1fs)',
                                   turned, p1_target, elapsed)
            if turned >= p1_target:
                rospy.loginfo('[UTURN/IMU] Phase1 완료')
                break
            if elapsed > UTURN_PHASE1_TO:
                rospy.logwarn('[UTURN/IMU] Phase1 타임아웃 (%.1f° 회전)', turned)
                break
            if self.is_emergency():
                self.stop(); return
            self.drive(SPEED_UTURN, 90)
            rate.sleep()

        self.stop()
        rospy.sleep(0.3)

        # Phase 2: 후진 + 좌회전 → 나머지 각도
        p1_turned = self.yaw_diff(p1_start, self.yaw_deg)
        remaining = max(UTURN_TARGET_DEG - p1_turned, 10.0)
        rospy.loginfo('[UTURN/IMU] Phase2: 후진+좌 (목표 %.1f°)', remaining)

        p2_start = self.yaw_deg
        t2 = rospy.Time.now()
        self._esc_reverse_init()  # 중립 1s → 후진 준비

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t2).to_sec()
            turned  = self.yaw_diff(p2_start, self.yaw_deg)
            rospy.loginfo_throttle(0.5, '[UTURN/IMU] P2 %.1f/%.1f° (%.1fs)',
                                   turned, remaining, elapsed)
            if turned >= remaining:
                rospy.loginfo('[UTURN/IMU] Phase2 완료')
                break
            if elapsed > UTURN_PHASE2_TO:
                rospy.logwarn('[UTURN/IMU] Phase2 타임아웃')
                break
            rear_d = min(self.sonar[5], self.sonar[6], self.sonar[7])
            if rear_d < SONAR_EMERGENCY:
                rospy.logwarn('[UTURN/IMU] 후방 장애물 중단')
                break
            self.drive(-SPEED_UTURN, -90)
            rate.sleep()

        self.stop()

    def _esc_reverse_init(self):
        rospy.loginfo('[UTURN] ESC 후진 준비 (중립 1s)')
        self.drive(0, 0)
        rospy.sleep(1.0)

    def _uturn_timed(self):
        rospy.loginfo('[UTURN/TIME] 시간 기반 시작')
        rate = rospy.Rate(20)
        for idx, (spd, ang, dur) in enumerate(UTURN_STEPS):
            rospy.loginfo('[UTURN/TIME] Step%d spd=%d ang=%d dur=%.1fs',
                          idx+1, spd, ang, dur)
            if spd < 0:
                self._esc_reverse_init()
            t0 = rospy.Time.now()
            while not rospy.is_shutdown():
                if (rospy.Time.now() - t0).to_sec() >= dur:
                    break
                if self.is_emergency():
                    self.stop(); return
                self.drive(spd, ang)
                rate.sleep()
            self.stop()
            rospy.sleep(0.2)
"""
