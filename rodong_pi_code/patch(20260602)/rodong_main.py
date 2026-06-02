#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rodong_main.py  —  RODONG13 rev3
================================================================
변경 (rev3):
  1. 회피 조향 유지 시간 증가 (AVOID_HOLD_MIN 0.8 → 1.5s)
  2. 회피/후진 후 IMU yaw 기반 원래 헤딩 복귀 (RECOVER 단계)
  3. 유턴 K-turn 개선: (조향)후진 → 전진 → (조향)후진 … IMU yaw 170°까지
  4. 전방 장애물 후진 시, 가장 가까운 전방 장애물 쪽으로 조향하며 후진
     (예: 우전방 장애물 → 우조향 후진 → 차 앞부분이 좌측으로 빠지며 clear)
  + STOP 명령이 유턴/후진/회피 중에도 즉시 반영되도록 abort 처리
  + 속도 상수 통합, 죽은 코드 제거

상태머신: IDLE → BUG_DRIVE ⇄ {MARKER_APPROACH → UTURN} / MANUAL_DRIVE / STOP
"""

import rospy
import math
import time
import threading
import tf
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Point
from xycar_msgs.msg import xycar_motor
import cv2
from cv_bridge import CvBridge

# ── ArUco ─────────────────────────────────────────────────────────────────
ARUCO_DICT   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
TARGET_ID    = 1

# ── 상태 ──────────────────────────────────────────────────────────────────
class State:
    IDLE            = 'IDLE'
    BUG_DRIVE       = 'BUG_DRIVE'
    MARKER_APPROACH = 'MARKER_APPROACH'
    UTURN           = 'UTURN'
    MANUAL_DRIVE    = 'MANUAL_DRIVE'
    STOP            = 'STOP'

# ── 회피 서브-단계 ─────────────────────────────────────────────────────────
class Avoid:
    NONE    = 'NONE'      # 일반 주행 (VFH 추종)
    STEER   = 'STEER'     # 회피 조향 유지
    RECOVER = 'RECOVER'   # 원래 헤딩으로 복귀

# ── 속도 [모터 단위] ───────────────────────────────────────────────────────
SPEED_DRIVE  = 25    # 일반/회피/접근/유턴 공통 주행 속도
SPEED_MANUAL = 20    # MANUAL 모드 속도

# ── 조향 [deg] ─────────────────────────────────────────────────────────────
ANGLE_MAX      = 90        # 물리 최대 조향
AVOID_FULL_ANG = 90        # 회피 풀조향

# ── 초음파 임계 [cm] ───────────────────────────────────────────────────────
SONAR_EMERGENCY = 15       # 전방 비상 (이하 → 후진)
SONAR_REVERSE   = 15       # 후진 중 후방 장애물 감지 거리

# ── 회피 로직 파라미터 ─────────────────────────────────────────────────────
AVOID_TRIG_ANG = 25        # |vfh_angle| 이 이상이면 회피 진입
AVOID_HOLD_MIN = 1.5       # [1] 회피 조향 최소 유지 시간 (s)  ← rev3 증가
AVOID_CLEAR_CM = 55        # 전방 이 이상이면 회피 해제 조건

# ── 헤딩 복귀(RECOVER) 파라미터 ───────────────────────────────────────────
RECOVER_TOL_DEG = 8.0      # [2] 이 오차 이내면 복귀 완료
RECOVER_GAIN    = 3.0      # yaw 오차 → 조향각 비례 게인
RECOVER_TIMEOUT = 4.0      # 복귀 단계 타임아웃 (s)

# ── 마커 ───────────────────────────────────────────────────────────────────
MARKER_CLOSE_PX = 80
MARKER_DEBOUNCE = 5

# ── MANUAL dead-reckoning ─────────────────────────────────────────────────
CM_PER_SEC_FWD  = 15.0
CM_PER_DEG_TURN = 0.12

# ── 유턴 (IMU K-turn) ──────────────────────────────────────────────────────
UTURN_TARGET_DEG = 170.0   # 누적 회전 목표 (180° - 마진)
UTURN_SEG_DEG    = 60.0    # 세그먼트당 회전 목표
UTURN_SEG_TO     = 4.0     # 세그먼트 타임아웃 (s)
UTURN_MAX_SEG    = 5        # 최대 세그먼트 수 (안전 상한)
# IMU 없을 때 시간 기반 fallback: (방향(+전진/-후진), 조향, 지속시간 s)
UTURN_TIMED_STEPS = [(-1, -90, 2.0), (+1, 90, 2.0), (-1, -90, 2.0)]


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


class RodongMain:
    def __init__(self):
        rospy.init_node('rodong_main', anonymous=False)

        self.state       = State.IDLE
        self.prev_state  = None
        self.sonar       = [999] * 8
        self.bridge      = CvBridge()
        self.frame       = None

        self.marker_seen = 0
        self.marker_cx   = -1
        self.marker_w    = 0

        self.vfh_speed   = 0
        self.vfh_angle   = 0

        # 회피 서브-FSM
        self.avoid_phase   = Avoid.NONE
        self.avoid_ang     = 0          # 래치된 회피 조향각
        self.avoid_t0      = None       # 회피 조향 시작 시각
        self.recover_t0    = None       # 복귀 시작 시각
        self.heading_target = None      # [2] 복귀할 원래 yaw [deg]

        self.manual_goal = None
        self.manual_done = False
        self.reversing   = False

        # IMU
        self.yaw_deg   = 0.0
        self.imu_ready = False

        # Publisher
        self.motor_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        # Subscribers
        rospy.Subscriber('/xycar_ultrasonic',   Int32MultiArray, self.cb_sonar)
        rospy.Subscriber('/usb_cam/image_raw',  Image,           self.cb_image)
        rospy.Subscriber('/rodong/vfh_cmd',     xycar_motor,     self.cb_vfh)
        rospy.Subscriber('/rodong/cmd',         String,          self.cb_cmd)
        rospy.Subscriber('/rodong/manual_goal', Point,           self.cb_manual_goal)
        rospy.Subscriber('/imu/data',           Imu,             self.cb_imu)

        rospy.loginfo('[Main] RODONG13 rev3 — IDLE. "i" 로 시작.')

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def cb_sonar(self, msg):
        self.sonar = list(msg.data)

    def cb_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5, '[Main] img err: %s', e)

    def cb_vfh(self, msg):
        self.vfh_speed = msg.speed
        self.vfh_angle = msg.angle

    def cb_imu(self, msg):
        q = msg.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_deg   = math.degrees(euler[2])
        self.imu_ready = True

    def cb_cmd(self, msg):
        cmd = msg.data.strip().upper()
        rospy.loginfo('[Main] CMD: %s', cmd)
        if cmd == 'INIT':
            if self.state in (State.IDLE, State.STOP):
                self.set_state(State.BUG_DRIVE)
            elif self.state == State.MANUAL_DRIVE:
                self.manual_goal = None
                self.set_state(State.BUG_DRIVE)
        elif cmd == 'STOP':
            self.set_state(State.STOP)     # 진행 중인 uturn/reverse/avoid 루프가 이 상태를 보고 abort
        elif cmd == 'UTURN':
            self.set_state(State.UTURN)
        elif cmd == 'MANUAL':
            self.set_state(State.MANUAL_DRIVE)
            self.manual_goal = None
            self.manual_done = True

    def cb_manual_goal(self, msg):
        if self.state == State.MANUAL_DRIVE:
            rospy.loginfo('[Main] 새 목표: x=%.1f y=%.1f cm', msg.x, msg.y)
            self.manual_goal = msg
            self.manual_done = False

    # ── 유틸 ──────────────────────────────────────────────────────────────
    def set_state(self, s):
        if s != self.state:
            rospy.loginfo('[Main] %s → %s', self.state, s)
            self.prev_state = self.state
            self.state = s
            # 상태 전환 시 회피 서브-FSM 리셋
            self._reset_avoid()

    def _reset_avoid(self):
        self.avoid_phase    = Avoid.NONE
        self.avoid_ang      = 0
        self.avoid_t0       = None
        self.recover_t0     = None
        self.heading_target = None

    def drive(self, speed, angle):
        msg = xycar_motor()
        msg.speed = int(speed)
        msg.angle = int(_clip(angle, -ANGLE_MAX, ANGLE_MAX))
        self.motor_pub.publish(msg)

    def stop_motor(self):
        self.drive(0, 0)

    def _esc_reverse_init(self):
        """후진 ESC deadband 대응: 중립 1s 후 후진 커맨드 가능."""
        self.drive(0, 0)
        rospy.sleep(1.0)

    def _front_min_cm(self):
        vals = [d for d in (self.sonar[1], self.sonar[2], self.sonar[3]) if d > 0]
        return min(vals) if vals else 999

    def _rear_min_cm(self):
        vals = [d for d in (self.sonar[5], self.sonar[6], self.sonar[7]) if d > 0]
        return min(vals) if vals else 999

    def is_front_blocked(self):
        return any(0 < d < SONAR_EMERGENCY for d in
                   (self.sonar[1], self.sonar[2], self.sonar[3]))

    def yaw_diff(self, start, current):
        """두 yaw 사이의 절대 차이 (0~180°)."""
        d = abs(current - start) % 360
        return d if d <= 180 else 360 - d

    def yaw_signed_diff(self, target, current):
        """target - current 를 (-180, 180]° 로 정규화.
        부호: + → yaw 증가 필요(좌회전), - → yaw 감소 필요(우회전)."""
        return (target - current + 180) % 360 - 180

    # ══════════════════════════════════════════════════════════════════════
    # [4] 후진 — 가장 가까운 전방 장애물 쪽으로 조향하며 후진
    # ══════════════════════════════════════════════════════════════════════
    def _reverse_steer_from_front(self):
        """전방에서 가장 가까운 장애물 쪽으로 조향각을 정한다.
        후진 시 차 앞부분이 그 반대쪽으로 빠지며 장애물에서 멀어진다."""
        lf = self.sonar[1] if self.sonar[1] > 0 else 999   # 좌전 (-45°)
        rf = self.sonar[2] if self.sonar[2] > 0 else 999   # 우전 (+45°)
        cf = self.sonar[3] if self.sonar[3] > 0 else 999   # 정면 ( 0°)

        if rf <= lf and rf <= cf:
            rospy.loginfo('[Reverse] 우전방(%.0fcm) → 우조향 후진(+%d°)', rf, AVOID_FULL_ANG)
            return AVOID_FULL_ANG
        if lf < rf and lf <= cf:
            rospy.loginfo('[Reverse] 좌전방(%.0fcm) → 좌조향 후진(-%d°)', lf, AVOID_FULL_ANG)
            return -AVOID_FULL_ANG
        # 정면 중앙 장애물 → 더 가까운 전방 쪽으로
        steer = AVOID_FULL_ANG if rf <= lf else -AVOID_FULL_ANG
        rospy.loginfo('[Reverse] 정면(%.0fcm) → steer %d°', cf, steer)
        return steer

    def reverse_motor(self, speed=SPEED_DRIVE, target_front_cm=30):
        """전방이 target_front_cm 확보될 때까지 후진. (별도 스레드에서 실행)
        장애물 쪽으로 조향하며 후진하고, 후방 장애물은 회피한다.
        완료 후 IMU 헤딩 복귀를 예약한다 [2]."""
        self.reversing = True
        steer = self._reverse_steer_from_front()
        rospy.loginfo('[Reverse] 후진 시작 — 전방 %.0fcm까지, steer=%d°',
                      target_front_cm, steer)

        # ESC 후진 활성화 시퀀스 (deadband 대응)
        self.drive(0, 0);             time.sleep(0.5)
        self.drive(-abs(speed), 0);   time.sleep(0.5)
        self.drive(0, 0);             time.sleep(0.5)

        rate    = rospy.Rate(20)
        timeout = time.time() + 8.0

        while (not rospy.is_shutdown()
               and self.state not in (State.STOP, State.IDLE)
               and time.time() < timeout):

            front = self._front_min_cm()
            if front >= target_front_cm:
                rospy.loginfo('[Reverse] 전방 %.0fcm 확보 → 완료', front)
                break

            rear_c = self.sonar[6]   # 후 (180°)
            rear_r = self.sonar[5]   # 우후 (+135°)
            rear_l = self.sonar[7]   # 좌후 (-135°)

            if 0 < rear_c < SONAR_REVERSE:
                rospy.logwarn('[Reverse] 후방 중앙 막힘 → 정지')
                break

            # 후진 조향은 전방 장애물 회피용으로 유지하되,
            # 조향하는 쪽 후방이 막히면 직진 후진으로 완화
            cur = steer
            if steer > 0 and 0 < rear_r < SONAR_REVERSE:
                cur = 0
            elif steer < 0 and 0 < rear_l < SONAR_REVERSE:
                cur = 0

            self.drive(-abs(speed), cur)
            rate.sleep()

        self.stop_motor()
        self.reversing = False

        # [2] 후진으로 틀어진 헤딩을 원래대로 복귀 예약
        if (self.imu_ready and self.heading_target is not None
                and self.state == State.BUG_DRIVE):
            self.avoid_phase = Avoid.RECOVER
            self.recover_t0  = rospy.Time.now()
            rospy.loginfo('[Reverse] 종료 → 헤딩 복귀 (target=%.1f°)', self.heading_target)
        else:
            rospy.loginfo('[Reverse] 종료')

    # ══════════════════════════════════════════════════════════════════════
    # [3] UTURN — IMU K-turn  ( (조향)후진 → 전진 → (조향)후진 … )
    # ══════════════════════════════════════════════════════════════════════
    def execute_uturn(self):
        if self.imu_ready:
            self._uturn_kturn()
        else:
            rospy.logwarn('[UTURN] IMU 없음 → 시간 기반 fallback')
            self._uturn_timed()

        self.stop_motor()
        # STOP 등으로 중단된 게 아니라 정상 완료한 경우에만 주행 복귀
        if self.state == State.UTURN:
            rospy.loginfo('[UTURN] 완료 → BUG_DRIVE')
            self.set_state(State.BUG_DRIVE)

    def _uturn_kturn(self):
        """IMU yaw 누적이 UTURN_TARGET_DEG 에 도달할 때까지
        후진(조향) → 전진(반대 조향) 세그먼트를 번갈아 수행 (우회전 방향)."""
        rospy.loginfo('[UTURN/IMU] K-turn 시작 (목표 %.0f°)', UTURN_TARGET_DEG)
        rate  = rospy.Rate(20)
        start = self.yaw_deg
        # 우회전 방향: 후진은 좌조향(-90), 전진은 우조향(+90) → 같은 방향으로 회전
        seg_pattern = [(-1, -AVOID_FULL_ANG), (+1, +AVOID_FULL_ANG)]

        for seg_idx in range(UTURN_MAX_SEG):
            if self.state != State.UTURN:
                return
            total = self.yaw_diff(start, self.yaw_deg)
            if total >= UTURN_TARGET_DEG:
                break

            direction, steer = seg_pattern[seg_idx % 2]
            kind = '후진' if direction < 0 else '전진'
            rospy.loginfo('[UTURN/IMU] Seg%d: %s steer=%d° (누적 %.1f°)',
                          seg_idx + 1, kind, steer, total)

            if direction < 0:
                self._esc_reverse_init()

            seg_start = self.yaw_deg
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and self.state == State.UTURN:
                if (rospy.Time.now() - t0).to_sec() > UTURN_SEG_TO:
                    break
                if self.yaw_diff(start, self.yaw_deg) >= UTURN_TARGET_DEG:
                    break
                if self.yaw_diff(seg_start, self.yaw_deg) >= UTURN_SEG_DEG:
                    break
                # 진행 방향 장애물 체크
                if direction < 0 and self._rear_min_cm() < SONAR_REVERSE:
                    rospy.logwarn('[UTURN/IMU] 후방 막힘 → 세그먼트 종료')
                    break
                if direction > 0 and self.is_front_blocked():
                    rospy.logwarn('[UTURN/IMU] 전방 막힘 → 세그먼트 종료')
                    break

                self.drive(direction * SPEED_DRIVE, steer)
                rate.sleep()

            self.stop_motor()
            rospy.sleep(0.3)

        rospy.loginfo('[UTURN/IMU] 누적 회전 %.1f°', self.yaw_diff(start, self.yaw_deg))

    def _uturn_timed(self):
        """IMU 없을 때 시간 기반 fallback (후진 → 전진 → 후진)."""
        rate = rospy.Rate(20)
        for idx, (direction, steer, dur) in enumerate(UTURN_TIMED_STEPS):
            if self.state != State.UTURN:
                return
            rospy.loginfo('[UTURN/TIME] Step%d: dir=%d steer=%d dur=%.1fs',
                          idx + 1, direction, steer, dur)
            if direction < 0:
                self._esc_reverse_init()

            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and self.state == State.UTURN:
                if (rospy.Time.now() - t0).to_sec() >= dur:
                    break
                if direction > 0 and self.is_front_blocked():
                    rospy.logwarn('[UTURN/TIME] Step%d 전방 막힘', idx + 1)
                    break
                if direction < 0 and self._rear_min_cm() < SONAR_REVERSE:
                    rospy.logwarn('[UTURN/TIME] Step%d 후방 막힘', idx + 1)
                    break
                self.drive(direction * SPEED_DRIVE, steer)
                rate.sleep()

            self.stop_motor()
            rospy.sleep(0.2)

    # ══════════════════════════════════════════════════════════════════════
    # MANUAL DRIVE (좌표 기반 dead-reckoning)
    # ══════════════════════════════════════════════════════════════════════
    def execute_manual_drive(self):
        goal = self.manual_goal
        if goal is None or self.manual_done:
            self.stop_motor()
            return

        x, y = goal.x, goal.y
        rospy.loginfo('[Manual] 목표: x=%.1f y=%.1f cm', x, y)

        # Step 1: 회전 정렬
        if abs(y) > 2.0:
            angle_deg = _clip(math.degrees(math.atan2(y, max(abs(x), 1.0))),
                              -ANGLE_MAX, ANGLE_MAX)
            turn_time = abs(angle_deg) * CM_PER_DEG_TURN
            t0   = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < turn_time and not rospy.is_shutdown():
                if self.state != State.MANUAL_DRIVE or self.is_front_blocked():
                    self.stop_motor(); self.manual_done = True; return
                self.drive(SPEED_MANUAL, int(angle_deg))
                rate.sleep()
            self.drive(SPEED_MANUAL, 0)
            rospy.sleep(0.1)

        # Step 2: 직진
        dist = math.hypot(x, y)
        if dist > 2.0:
            drive_time = dist / CM_PER_SEC_FWD
            spd  = SPEED_MANUAL if x >= 0 else -SPEED_MANUAL
            t0   = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < drive_time and not rospy.is_shutdown():
                if self.state != State.MANUAL_DRIVE:
                    self.stop_motor(); self.manual_done = True; return
                if self.is_front_blocked() and spd > 0:
                    self.stop_motor(); self.manual_done = True; return
                self.drive(spd, 0)
                rate.sleep()

        self.stop_motor()
        rospy.loginfo('[Manual] 도달 (추정)')
        self.manual_done = True
        self.manual_goal = None

    # ══════════════════════════════════════════════════════════════════════
    # BUG_DRIVE — VFH 추종 + 회피 서브-FSM ([1][2])
    # ══════════════════════════════════════════════════════════════════════
    def get_avoid_steer_from_sonar(self):
        """전방 초음파 기준 회피 조향 (장애물 반대 방향으로 전진 회피)."""
        front_l = self.sonar[1]   # 좌전 (-45°)
        front_c = self.sonar[3]   # 정면 ( 0°)
        front_r = self.sonar[2]   # 우전 (+45°)

        valid = {}
        if 0 < front_l < 200: valid['left']   = front_l
        if 0 < front_c < 200: valid['center'] = front_c
        if 0 < front_r < 200: valid['right']  = front_r
        if not valid:
            return 0

        closest  = min(valid, key=valid.get)
        min_dist = valid[closest]

        if closest == 'left':
            rospy.loginfo('[Sonar] 좌전(%.0fcm) → 우조향(+90°)', min_dist)
            return AVOID_FULL_ANG
        if closest == 'right':
            rospy.loginfo('[Sonar] 우전(%.0fcm) → 좌조향(-90°)', min_dist)
            return -AVOID_FULL_ANG
        # 정면이 가장 가까움 → 더 트인 쪽으로
        if valid.get('left', 999) < valid.get('right', 999):
            rospy.loginfo('[Sonar] 정면(%.0fcm) 좌측 더 막힘 → 우조향', min_dist)
            return AVOID_FULL_ANG
        rospy.loginfo('[Sonar] 정면(%.0fcm) 우측 더 막힘 → 좌조향', min_dist)
        return -AVOID_FULL_ANG

    def _bug_drive_step(self):
        now   = rospy.Time.now()
        front = self._front_min_cm()

        # ── NONE: 일반 주행, 회피 진입 판정 ──────────────────────────────
        if self.avoid_phase == Avoid.NONE:
            sonar_close = front < SONAR_EMERGENCY * 1.5
            vfh_sharp   = abs(self.vfh_angle) >= AVOID_TRIG_ANG
            if sonar_close or vfh_sharp:
                self.avoid_phase = Avoid.STEER
                self.avoid_ang   = (self.get_avoid_steer_from_sonar() if sonar_close
                                    else (AVOID_FULL_ANG if self.vfh_angle > 0
                                          else -AVOID_FULL_ANG))
                self.avoid_t0 = now
                if self.imu_ready and self.heading_target is None:
                    self.heading_target = self.yaw_deg   # [2] 진입 시 원래 헤딩 저장
                rospy.loginfo('[Avoid] 진입 (steer=%d°, head0=%s)',
                              self.avoid_ang,
                              '%.1f' % self.heading_target if self.heading_target is not None else 'NA')
            else:
                self.drive(self.vfh_speed, self.vfh_angle)
                return

        # ── STEER: 회피 조향 유지 ([1] 더 오래) ──────────────────────────
        if self.avoid_phase == Avoid.STEER:
            self.drive(self.vfh_speed, self.avoid_ang)
            held = (now - self.avoid_t0).to_sec()
            if held >= AVOID_HOLD_MIN and front >= AVOID_CLEAR_CM:
                if self.imu_ready and self.heading_target is not None:
                    self.avoid_phase = Avoid.RECOVER
                    self.recover_t0  = now
                    rospy.loginfo('[Avoid] 조향 해제 → 헤딩 복귀 (front=%.0fcm)', front)
                else:
                    self._reset_avoid()
                    rospy.loginfo('[Avoid] 해제 (IMU 없음, front=%.0fcm)', front)
            return

        # ── RECOVER: 원래 헤딩으로 복귀 ([2]) ────────────────────────────
        if self.avoid_phase == Avoid.RECOVER:
            # 복귀 중 다시 장애물 → 회피로 회귀
            if front < SONAR_EMERGENCY * 1.5:
                self.avoid_phase = Avoid.STEER
                self.avoid_ang   = self.get_avoid_steer_from_sonar()
                self.avoid_t0    = now
                rospy.loginfo('[Recover] 장애물 재감지 → 회피 복귀')
                return

            err = self.yaw_signed_diff(self.heading_target, self.yaw_deg)
            if abs(err) <= RECOVER_TOL_DEG or \
               (now - self.recover_t0).to_sec() > RECOVER_TIMEOUT:
                rospy.loginfo('[Recover] 완료 (오차 %.1f°)', err)
                self._reset_avoid()
                self.drive(self.vfh_speed, self.vfh_angle)
                return

            # err>0 → yaw 증가 필요(좌회전, 음의 조향), err<0 → 우회전(양의 조향)
            steer = int(_clip(-RECOVER_GAIN * err, -ANGLE_MAX, ANGLE_MAX))
            self.drive(self.vfh_speed, steer)
            return

    # ══════════════════════════════════════════════════════════════════════
    # 메인 루프
    # ══════════════════════════════════════════════════════════════════════
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.state in (State.IDLE, State.STOP):
                self.stop_motor(); rate.sleep(); continue

            if self.state == State.MANUAL_DRIVE:
                self.execute_manual_drive(); rate.sleep(); continue

            if self.state == State.UTURN:
                self.execute_uturn(); continue

            # ── BUG_DRIVE / MARKER_APPROACH 공통: 비상 후진 ──────────────
            if self.reversing:
                rate.sleep(); continue
            if self.is_front_blocked():
                rospy.logwarn_throttle(1, '[Main] EMERGENCY → 후진')
                if self.imu_ready and self.heading_target is None:
                    self.heading_target = self.yaw_deg   # [2] 후진 전 헤딩 저장
                t = threading.Thread(target=self.reverse_motor,
                                     kwargs={'speed': SPEED_DRIVE, 'target_front_cm': 30})
                t.daemon = True
                t.start()
                rate.sleep(); continue

            if self.state == State.MARKER_APPROACH:
                if self.detect_marker():
                    if self.marker_w >= MARKER_CLOSE_PX:
                        rospy.loginfo('[Main] 마커 접근 완료 → UTURN')
                        self.stop_motor()
                        self.set_state(State.UTURN)
                    else:
                        frame_w = self.frame.shape[1] if self.frame is not None else 640
                        err  = self.marker_cx - frame_w // 2
                        gain = 150.0 / (frame_w // 2)
                        self.drive(SPEED_DRIVE, int(err * gain))
                else:
                    self.drive(SPEED_DRIVE, 0)
                rate.sleep(); continue

            if self.state == State.BUG_DRIVE:
                if self.detect_marker():
                    self.marker_seen += 1
                else:
                    self.marker_seen = 0
                if self.marker_seen >= MARKER_DEBOUNCE:
                    rospy.loginfo('[Main] 마커 → MARKER_APPROACH')
                    self.marker_seen = 0
                    self.set_state(State.MARKER_APPROACH)
                    continue
                self._bug_drive_step()

            rate.sleep()

    def detect_marker(self):
        if self.frame is None:
            return False
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if mid == TARGET_ID:
                    pts = corners[i][0]
                    self.marker_cx = int(pts[:, 0].mean())
                    self.marker_w  = int(pts[:, 0].max() - pts[:, 0].min())
                    rospy.loginfo_throttle(0.5, '[ArUco] marker_w=%dpx  marker_cx=%dpx',
                                           self.marker_w, self.marker_cx)
                    return True
        return False


if __name__ == '__main__':
    try:
        RodongMain().run()
    except rospy.ROSInterruptException:
        pass
