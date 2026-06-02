#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rodong_main.py  —  RODONG13 rev4
================================================================
변경 (rev4):
  + 공용 설정/기하/초음파 로직을 rodong_config / rodong_geometry / rodong_sonar 로 분리
    (vfh_planner 와 상수 단일 출처, 단위테스트 가능).
  + ArUco 중복 검출 제거: 더 이상 /usb_cam/image_raw 를 직접 디코드/검출하지 않고
    aruco_detector 가 발행하는 /aruco_pose 를 구독 (Pi CPU 절감).
  + 폐루프 제어 PID 화: 마커 접근(bearing→조향), 헤딩 복귀(yaw오차→조향).

이전 (rev3):
  1. 회피 조향 유지 시간 증가 / 2. IMU yaw 헤딩 복귀(RECOVER)
  3. 유턴 K-turn / 4. 전방 장애물 쪽 조향 후진 / STOP abort

상태머신: IDLE → BUG_DRIVE ⇄ {MARKER_APPROACH → UTURN} / MANUAL_DRIVE / STOP
"""

import os
import sys
import math
import time
import threading
import rospy
import tf
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, PoseStamped
from xycar_msgs.msg import xycar_motor

# catkin 은 devel/lib 래퍼에서 노드를 실행하므로 scripts/ 가 sys.path 에 없음.
# 동일 폴더의 공용 모듈(rodong_config 등)을 import 하도록 경로 추가.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rodong_config as cfg
from rodong_geometry import clip, yaw_diff, yaw_signed_diff
from rodong_sonar import front_min, rear_min
from rodong_control import PID


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


class RodongMain:
    def __init__(self):
        rospy.init_node('rodong_main', anonymous=False)

        self.state       = State.IDLE
        self.prev_state  = None
        self.sonar       = [999] * 8

        # 마커 (/aruco_pose 구독 — 직접 검출 안 함)
        self.marker_bearing  = 0.0     # [rad] 좌(-)/우(+)
        self.marker_pixel_w  = 0.0     # [px]
        self.last_marker_t   = None
        self.marker_seen     = 0

        self.vfh_speed   = 0
        self.vfh_angle   = 0

        # 회피 서브-FSM
        self.avoid_phase   = Avoid.NONE
        self.avoid_ang     = 0          # 래치된 회피 조향각
        self.avoid_t0      = None       # 회피 조향 시작 시각
        self.recover_t0    = None       # 복귀 시작 시각
        self.heading_target = None      # 복귀할 원래 yaw [deg]

        self.manual_goal = None
        self.manual_done = False
        self.reversing   = False

        # IMU
        self.yaw_deg   = 0.0
        self.imu_ready = False

        # ── PID 제어기 ──
        self.marker_pid  = PID(**cfg.MARKER_PID)    # bearing[rad] → 조향[deg]
        self.recover_pid = PID(**cfg.RECOVER_PID)   # yaw오차[deg] → 조향[deg]
        self._marker_prev_t  = None
        self._recover_prev_t = None

        # Publisher
        self.motor_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        # Subscribers
        rospy.Subscriber('/xycar_ultrasonic',   Int32MultiArray, self.cb_sonar)
        rospy.Subscriber('/aruco_pose',         PoseStamped,     self.cb_aruco)
        rospy.Subscriber('/rodong/vfh_cmd',     xycar_motor,     self.cb_vfh)
        rospy.Subscriber('/rodong/cmd',         String,          self.cb_cmd)
        rospy.Subscriber('/rodong/manual_goal', Point,           self.cb_manual_goal)
        rospy.Subscriber('/imu/data',           Imu,             self.cb_imu)

        rospy.loginfo('[Main] RODONG13 rev4 — IDLE. "i" 로 시작.')

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def cb_sonar(self, msg):
        self.sonar = list(msg.data)

    def cb_aruco(self, msg):
        # aruco_detector 가 마커 검출 시에만 발행 → 수신 = 보임.
        self.marker_bearing = msg.pose.orientation.z   # [rad]
        self.marker_pixel_w = msg.pose.position.z      # [px]
        self.last_marker_t  = rospy.Time.now()

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
            # 상태 전환 시 회피 서브-FSM + 제어기 리셋
            self._reset_avoid()
            self.marker_pid.reset()
            self._marker_prev_t = None

    def _reset_avoid(self):
        self.avoid_phase    = Avoid.NONE
        self.avoid_ang      = 0
        self.avoid_t0       = None
        self.recover_t0     = None
        self.heading_target = None
        self.recover_pid.reset()
        self._recover_prev_t = None

    def _valid(self, t, timeout):
        return t is not None and (rospy.Time.now() - t).to_sec() < timeout

    def marker_visible(self):
        return self._valid(self.last_marker_t, cfg.MARKER_TIMEOUT)

    def _dt(self, attr):
        """attr 에 저장된 직전 시각 대비 경과[s]. 첫 호출은 루프주기(0.05)."""
        now  = rospy.Time.now()
        prev = getattr(self, attr)
        setattr(self, attr, now)
        return (now - prev).to_sec() if prev is not None else 0.05

    def drive(self, speed, angle):
        msg = xycar_motor()
        msg.speed = int(speed)
        msg.angle = int(clip(angle, -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
        self.motor_pub.publish(msg)

    def stop_motor(self):
        self.drive(0, 0)

    def _esc_reverse_init(self):
        """후진 ESC deadband 대응: 중립 1s 후 후진 커맨드 가능."""
        self.drive(0, 0)
        rospy.sleep(1.0)

    def is_front_blocked(self):
        return any(0 < d < cfg.SONAR_EMERGENCY for d in
                   (self.sonar[cfg.SONAR_FRONT_L], self.sonar[cfg.SONAR_FRONT_R],
                    self.sonar[cfg.SONAR_FRONT]))

    # ══════════════════════════════════════════════════════════════════════
    # 후진 — 가장 가까운 전방 장애물 쪽으로 조향하며 후진
    # ══════════════════════════════════════════════════════════════════════
    def _reverse_steer_from_front(self):
        """전방에서 가장 가까운 장애물 쪽으로 조향각을 정한다.
        후진 시 차 앞부분이 그 반대쪽으로 빠지며 장애물에서 멀어진다."""
        lf = self.sonar[cfg.SONAR_FRONT_L] if self.sonar[cfg.SONAR_FRONT_L] > 0 else 999
        rf = self.sonar[cfg.SONAR_FRONT_R] if self.sonar[cfg.SONAR_FRONT_R] > 0 else 999
        cf = self.sonar[cfg.SONAR_FRONT]   if self.sonar[cfg.SONAR_FRONT]   > 0 else 999

        if rf <= lf and rf <= cf:
            rospy.loginfo('[Reverse] 우전방(%.0fcm) → 우조향 후진(+%d°)', rf, cfg.AVOID_FULL_ANG)
            return cfg.AVOID_FULL_ANG
        if lf < rf and lf <= cf:
            rospy.loginfo('[Reverse] 좌전방(%.0fcm) → 좌조향 후진(-%d°)', lf, cfg.AVOID_FULL_ANG)
            return -cfg.AVOID_FULL_ANG
        # 정면 중앙 장애물 → 더 가까운 전방 쪽으로
        steer = cfg.AVOID_FULL_ANG if rf <= lf else -cfg.AVOID_FULL_ANG
        rospy.loginfo('[Reverse] 정면(%.0fcm) → steer %d°', cf, steer)
        return steer

    def reverse_motor(self, speed=None, target_front_cm=30):
        """전방이 target_front_cm 확보될 때까지 후진. (별도 스레드에서 실행)
        장애물 쪽으로 조향하며 후진하고, 후방 장애물은 회피한다.
        완료 후 IMU 헤딩 복귀를 예약한다."""
        if speed is None:
            speed = cfg.SPEED_DRIVE
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

            front = front_min(self.sonar)
            if front >= target_front_cm:
                rospy.loginfo('[Reverse] 전방 %.0fcm 확보 → 완료', front)
                break

            rear_c = self.sonar[cfg.SONAR_REAR]     # 후 (180°)
            rear_r = self.sonar[cfg.SONAR_REAR_R]   # 우후 (+135°)
            rear_l = self.sonar[cfg.SONAR_REAR_L]   # 좌후 (-135°)

            if 0 < rear_c < cfg.SONAR_REVERSE:
                rospy.logwarn('[Reverse] 후방 중앙 막힘 → 정지')
                break

            # 후진 조향은 전방 장애물 회피용으로 유지하되,
            # 조향하는 쪽 후방이 막히면 직진 후진으로 완화
            cur = steer
            if steer > 0 and 0 < rear_r < cfg.SONAR_REVERSE:
                cur = 0
            elif steer < 0 and 0 < rear_l < cfg.SONAR_REVERSE:
                cur = 0

            self.drive(-abs(speed), cur)
            rate.sleep()

        self.stop_motor()
        self.reversing = False

        # 후진으로 틀어진 헤딩을 원래대로 복귀 예약
        if (self.imu_ready and self.heading_target is not None
                and self.state == State.BUG_DRIVE):
            self.avoid_phase = Avoid.RECOVER
            self.recover_t0  = rospy.Time.now()
            self.recover_pid.reset()
            self._recover_prev_t = None
            rospy.loginfo('[Reverse] 종료 → 헤딩 복귀 (target=%.1f°)', self.heading_target)
        else:
            rospy.loginfo('[Reverse] 종료')

    # ══════════════════════════════════════════════════════════════════════
    # UTURN — IMU K-turn  ( (조향)후진 → 전진 → (조향)후진 … )
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
        rospy.loginfo('[UTURN/IMU] K-turn 시작 (목표 %.0f°)', cfg.UTURN_TARGET_DEG)
        rate  = rospy.Rate(20)
        start = self.yaw_deg
        # 우회전 방향: 후진은 좌조향(-90), 전진은 우조향(+90) → 같은 방향으로 회전
        seg_pattern = [(-1, -cfg.AVOID_FULL_ANG), (+1, +cfg.AVOID_FULL_ANG)]

        for seg_idx in range(cfg.UTURN_MAX_SEG):
            if self.state != State.UTURN:
                return
            total = yaw_diff(start, self.yaw_deg)
            if total >= cfg.UTURN_TARGET_DEG:
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
                if (rospy.Time.now() - t0).to_sec() > cfg.UTURN_SEG_TO:
                    break
                if yaw_diff(start, self.yaw_deg) >= cfg.UTURN_TARGET_DEG:
                    break
                if yaw_diff(seg_start, self.yaw_deg) >= cfg.UTURN_SEG_DEG:
                    break
                # 진행 방향 장애물 체크
                if direction < 0 and rear_min(self.sonar) < cfg.SONAR_REVERSE:
                    rospy.logwarn('[UTURN/IMU] 후방 막힘 → 세그먼트 종료')
                    break
                if direction > 0 and self.is_front_blocked():
                    rospy.logwarn('[UTURN/IMU] 전방 막힘 → 세그먼트 종료')
                    break

                self.drive(direction * cfg.SPEED_DRIVE, steer)
                rate.sleep()

            self.stop_motor()
            rospy.sleep(0.3)

        rospy.loginfo('[UTURN/IMU] 누적 회전 %.1f°', yaw_diff(start, self.yaw_deg))

    def _uturn_timed(self):
        """IMU 없을 때 시간 기반 fallback (후진 → 전진 → 후진)."""
        rate = rospy.Rate(20)
        for idx, (direction, steer, dur) in enumerate(cfg.UTURN_TIMED_STEPS):
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
                if direction < 0 and rear_min(self.sonar) < cfg.SONAR_REVERSE:
                    rospy.logwarn('[UTURN/TIME] Step%d 후방 막힘', idx + 1)
                    break
                self.drive(direction * cfg.SPEED_DRIVE, steer)
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
            angle_deg = clip(math.degrees(math.atan2(y, max(abs(x), 1.0))),
                             -cfg.ANGLE_MAX, cfg.ANGLE_MAX)
            turn_time = abs(angle_deg) * cfg.CM_PER_DEG_TURN
            t0   = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < turn_time and not rospy.is_shutdown():
                if self.state != State.MANUAL_DRIVE or self.is_front_blocked():
                    self.stop_motor(); self.manual_done = True; return
                self.drive(cfg.SPEED_MANUAL, int(angle_deg))
                rate.sleep()
            self.drive(cfg.SPEED_MANUAL, 0)
            rospy.sleep(0.1)

        # Step 2: 직진
        dist = math.hypot(x, y)
        if dist > 2.0:
            drive_time = dist / cfg.CM_PER_SEC_FWD
            spd  = cfg.SPEED_MANUAL if x >= 0 else -cfg.SPEED_MANUAL
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
    # BUG_DRIVE — VFH 추종 + 회피 서브-FSM
    # ══════════════════════════════════════════════════════════════════════
    def get_avoid_steer_from_sonar(self):
        """전방 초음파 기준 회피 조향 (장애물 반대 방향으로 전진 회피)."""
        front_l = self.sonar[cfg.SONAR_FRONT_L]   # 좌전 (-45°)
        front_c = self.sonar[cfg.SONAR_FRONT]     # 정면 ( 0°)
        front_r = self.sonar[cfg.SONAR_FRONT_R]   # 우전 (+45°)

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
            return cfg.AVOID_FULL_ANG
        if closest == 'right':
            rospy.loginfo('[Sonar] 우전(%.0fcm) → 좌조향(-90°)', min_dist)
            return -cfg.AVOID_FULL_ANG
        # 정면이 가장 가까움 → 더 트인 쪽으로
        if valid.get('left', 999) < valid.get('right', 999):
            rospy.loginfo('[Sonar] 정면(%.0fcm) 좌측 더 막힘 → 우조향', min_dist)
            return cfg.AVOID_FULL_ANG
        rospy.loginfo('[Sonar] 정면(%.0fcm) 우측 더 막힘 → 좌조향', min_dist)
        return -cfg.AVOID_FULL_ANG

    def _bug_drive_step(self):
        now   = rospy.Time.now()
        front = front_min(self.sonar)

        # ── NONE: 일반 주행, 회피 진입 판정 ──────────────────────────────
        if self.avoid_phase == Avoid.NONE:
            sonar_close = front < cfg.SONAR_EMERGENCY * 1.5
            vfh_sharp   = abs(self.vfh_angle) >= cfg.AVOID_TRIG_ANG
            if sonar_close or vfh_sharp:
                self.avoid_phase = Avoid.STEER
                self.avoid_ang   = (self.get_avoid_steer_from_sonar() if sonar_close
                                    else (cfg.AVOID_FULL_ANG if self.vfh_angle > 0
                                          else -cfg.AVOID_FULL_ANG))
                self.avoid_t0 = now
                if self.imu_ready and self.heading_target is None:
                    self.heading_target = self.yaw_deg   # 진입 시 원래 헤딩 저장
                rospy.loginfo('[Avoid] 진입 (steer=%d°, head0=%s)',
                              self.avoid_ang,
                              '%.1f' % self.heading_target if self.heading_target is not None else 'NA')
            else:
                self.drive(self.vfh_speed, self.vfh_angle)
                return

        # ── STEER: 회피 조향 유지 ────────────────────────────────────────
        if self.avoid_phase == Avoid.STEER:
            self.drive(self.vfh_speed, self.avoid_ang)
            held = (now - self.avoid_t0).to_sec()
            if held >= cfg.AVOID_HOLD_MIN and front >= cfg.AVOID_CLEAR_CM:
                if self.imu_ready and self.heading_target is not None:
                    self.avoid_phase = Avoid.RECOVER
                    self.recover_t0  = now
                    self.recover_pid.reset()
                    self._recover_prev_t = None
                    rospy.loginfo('[Avoid] 조향 해제 → 헤딩 복귀 (front=%.0fcm)', front)
                else:
                    self._reset_avoid()
                    rospy.loginfo('[Avoid] 해제 (IMU 없음, front=%.0fcm)', front)
            return

        # ── RECOVER: 원래 헤딩으로 복귀 (PID) ────────────────────────────
        if self.avoid_phase == Avoid.RECOVER:
            # 복귀 중 다시 장애물 → 회피로 회귀
            if front < cfg.SONAR_EMERGENCY * 1.5:
                self.avoid_phase = Avoid.STEER
                self.avoid_ang   = self.get_avoid_steer_from_sonar()
                self.avoid_t0    = now
                rospy.loginfo('[Recover] 장애물 재감지 → 회피 복귀')
                return

            err = yaw_signed_diff(self.heading_target, self.yaw_deg)
            if abs(err) <= cfg.RECOVER_TOL_DEG or \
               (now - self.recover_t0).to_sec() > cfg.RECOVER_TIMEOUT:
                rospy.loginfo('[Recover] 완료 (오차 %.1f°)', err)
                self._reset_avoid()
                self.drive(self.vfh_speed, self.vfh_angle)
                return

            # err>0 → yaw 증가 필요(좌회전, 음의 조향). PID 출력에 부호 반전.
            dt = self._dt('_recover_prev_t')
            steer = int(clip(-self.recover_pid.step(err, dt), -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
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
                    self.heading_target = self.yaw_deg   # 후진 전 헤딩 저장
                t = threading.Thread(target=self.reverse_motor,
                                     kwargs={'speed': cfg.SPEED_DRIVE, 'target_front_cm': 30})
                t.daemon = True
                t.start()
                rate.sleep(); continue

            if self.state == State.MARKER_APPROACH:
                if self.marker_visible():
                    if self.marker_pixel_w >= cfg.MARKER_CLOSE_PX:
                        rospy.loginfo('[Main] 마커 접근 완료(%.0fpx) → UTURN', self.marker_pixel_w)
                        self.stop_motor()
                        self.set_state(State.UTURN)
                    else:
                        # bearing(rad) 오차 → PID 조향 (우측 마커=+bearing=우조향)
                        dt = self._dt('_marker_prev_t')
                        steer = int(clip(self.marker_pid.step(self.marker_bearing, dt),
                                         -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
                        self.drive(cfg.SPEED_DRIVE, steer)
                else:
                    self.drive(cfg.SPEED_DRIVE, 0)
                rate.sleep(); continue

            if self.state == State.BUG_DRIVE:
                if self.marker_visible():
                    self.marker_seen += 1
                else:
                    self.marker_seen = 0
                if self.marker_seen >= cfg.MARKER_DEBOUNCE:
                    rospy.loginfo('[Main] 마커 → MARKER_APPROACH')
                    self.marker_seen = 0
                    self.set_state(State.MARKER_APPROACH)
                    continue
                self._bug_drive_step()

            rate.sleep()


if __name__ == '__main__':
    try:
        RodongMain().run()
    except rospy.ROSInterruptException:
        pass
