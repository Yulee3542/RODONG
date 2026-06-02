#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
uturn_patch.py
Pi에서 실행:  python3 uturn_patch.py
rodong_main.py의 유턴 관련 상수 + execute_uturn() 전체를 교체한다.
"""

import re, sys, os

TARGET = os.path.expanduser(
    '~/xycar_ws/src/rodong/scripts/rodong_main.py')

# ─────────────────────────────────────────────────────────────────────────────
# 교체할 상수 블록 (원본에서 UTURN 관련 상수가 있는 부분)
# ─────────────────────────────────────────────────────────────────────────────
OLD_CONST_PAT = re.compile(
    r'(# ── IMU 기반 UTURN.*?USE_IMU_UTURN\s*=\s*\S+.*?UTURN_TIMEOUT\s*=\s*\S+)',
    re.DOTALL
)

NEW_CONST = """\
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
]"""

# ─────────────────────────────────────────────────────────────────────────────
# 교체할 함수 블록 (yaw_diff + execute_uturn 전체)
# ─────────────────────────────────────────────────────────────────────────────
OLD_FUNC_PAT = re.compile(
    r'(def yaw_diff\(self.*?)(?=\n    def |\nclass |\Z)',
    re.DOTALL
)

NEW_FUNC = '''\
def yaw_diff(self, start, current):
        """두 yaw 각도 사이의 절대 차이 (0~180°)"""
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

    # ── IMU 기반 유턴 ─────────────────────────────────────────────────────
    def _uturn_imu(self):
        rospy.loginfo('[UTURN/IMU] 시작')
        rate = rospy.Rate(20)

        # ── Phase 1: 전진 + 우회전 → 90° 목표 ────────────────────────────
        rospy.loginfo('[UTURN/IMU] Phase1: 전진+우90°')
        p1_start  = self.yaw_deg
        p1_target = UTURN_TARGET_DEG / 2.0   # ≈ 87.5°
        t1 = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t1).to_sec()
            turned  = self.yaw_diff(p1_start, self.yaw_deg)
            rospy.loginfo_throttle(0.5, '[UTURN/IMU] P1 %.1f° / %.1f° (%.1fs)',
                                   turned, p1_target, elapsed)

            if turned >= p1_target:
                rospy.loginfo('[UTURN/IMU] Phase1 각도 도달')
                break
            if elapsed > UTURN_PHASE1_TO:
                rospy.logwarn('[UTURN/IMU] Phase1 타임아웃 (%.1f° 회전)', turned)
                break
            if self.is_emergency():
                rospy.logwarn('[UTURN/IMU] Phase1 비상정지')
                self.stop()
                return

            self.drive(SPEED_UTURN, 90)
            rate.sleep()

        self.stop()
        rospy.sleep(0.3)

        # ── Phase 2: 후진 + 좌회전 → 나머지 각도 ─────────────────────────
        rospy.loginfo('[UTURN/IMU] Phase2: 후진+좌 (나머지 각도)')
        p1_turned = self.yaw_diff(p1_start, self.yaw_deg)
        remaining = max(UTURN_TARGET_DEG - p1_turned, 10.0)
        rospy.loginfo('[UTURN/IMU] Phase1 실제 %.1f° → Phase2 목표 %.1f°',
                      p1_turned, remaining)

        p2_start = self.yaw_deg
        t2 = rospy.Time.now()

        # 후진 ESC 시퀀스: 중립 → 후진 명령
        self._esc_reverse_init()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t2).to_sec()
            turned  = self.yaw_diff(p2_start, self.yaw_deg)
            rospy.loginfo_throttle(0.5, '[UTURN/IMU] P2 %.1f° / %.1f° (%.1fs)',
                                   turned, remaining, elapsed)

            if turned >= remaining:
                rospy.loginfo('[UTURN/IMU] Phase2 각도 도달')
                break
            if elapsed > UTURN_PHASE2_TO:
                rospy.logwarn('[UTURN/IMU] Phase2 타임아웃 (%.1f° 회전)', turned)
                break
            # 후방 장애물 체크
            rear_d = min(self.sonar[6], self.sonar[5], self.sonar[7])
            if rear_d < SONAR_EMERGENCY:
                rospy.logwarn('[UTURN/IMU] Phase2 후방 장애물 → 중단')
                break

            self.drive(-SPEED_UTURN, -90)
            rate.sleep()

        self.stop()

    # ── 후진 ESC 시퀀스 ───────────────────────────────────────────────────
    def _esc_reverse_init(self):
        """중립 1s → 후진 커맨드 전송 (ESC deadband 대응)"""
        rospy.loginfo('[UTURN] ESC 후진 시퀀스')
        self.drive(0, 0)
        rospy.sleep(1.0)

    # ── 시간 기반 fallback ────────────────────────────────────────────────
    def _uturn_timed(self):
        rospy.loginfo('[UTURN/TIME] UTURN_STEPS 기반 시작')
        rate = rospy.Rate(20)

        for idx, (spd, ang, dur) in enumerate(UTURN_STEPS):
            rospy.loginfo('[UTURN/TIME] Step%d: spd=%d ang=%d dur=%.1fs',
                          idx + 1, spd, ang, dur)

            # 후진 스텝이면 ESC 시퀀스 먼저
            if spd < 0:
                self._esc_reverse_init()

            t0 = rospy.Time.now()
            while not rospy.is_shutdown():
                if (rospy.Time.now() - t0).to_sec() >= dur:
                    break
                if self.is_emergency():
                    rospy.logwarn('[UTURN/TIME] Step%d 비상정지', idx + 1)
                    self.stop()
                    return
                self.drive(spd, ang)
                rate.sleep()

            self.stop()
            rospy.sleep(0.2)

'''

# ─────────────────────────────────────────────────────────────────────────────
# 적용
# ─────────────────────────────────────────────────────────────────────────────
def apply_patch():
    with open(TARGET, 'r') as f:
        code = f.read()

    # 백업
    bak = TARGET + '.bak_uturn'
    with open(bak, 'w') as f:
        f.write(code)
    print(f'[backup] {bak}')

    changed = False

    # 1) 상수 블록 교체
    m = OLD_CONST_PAT.search(code)
    if m:
        code = code[:m.start()] + NEW_CONST + code[m.end():]
        print('[OK] 상수 블록 교체 완료')
        changed = True
    else:
        print('[WARN] 상수 블록을 찾지 못했습니다 — 수동으로 추가하세요:')
        print(NEW_CONST)

    # 2) 함수 블록 교체
    m = OLD_FUNC_PAT.search(code)
    if m:
        code = code[:m.start()] + NEW_FUNC + code[m.end():]
        print('[OK] execute_uturn / yaw_diff 교체 완료')
        changed = True
    else:
        print('[WARN] yaw_diff/execute_uturn을 찾지 못했습니다 — 수동 삽입 필요')

    if changed:
        with open(TARGET, 'w') as f:
            f.write(code)
        print(f'[done] {TARGET} 저장 완료')
    else:
        print('[fail] 아무것도 교체되지 않았습니다 — 파일 직접 확인 필요')

if __name__ == '__main__':
    apply_patch()
