#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rodong_main.py  —  RODONG13 rev2
변경: 속도20, IMU yaw 기반 방향전환, 조향±90°
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

# ── 파라미터 ──────────────────────────────────────────────────────────────
SPEED_NORMAL   = 20
SPEED_APPROACH = 20
SPEED_UTURN    = 20
SPEED_MANUAL   = 20

ANGLE_CENTER   = 0
ANGLE_UTURN    = 90       # 최대 조향

SONAR_EMERGENCY  = 15      # cm
AVOID_FULL_ANG   = 50      # 회피 시 풀조향 각도
AVOID_HOLD_MIN   = 0.8     # 회피 조향 최소 유지 시간(s)
AVOID_TRIG_ANG   = 25      # vfh_angle 절댓값 이 이상이면 회피 진입
AVOID_CLEAR_CM   = 55      # 전방빔 이 이상이면 회피 해제
SONAR_REVERSE    = 15     # cm: 후진 중 후방 장애물 감지 거리
MARKER_CLOSE_PX  = 80
MARKER_DEBOUNCE  = 5

# dead reckoning
CM_PER_SEC_FWD  = 15.0
CM_PER_DEG_TURN = 0.12

# IMU 방향전환
UTURN_TARGET_DEG  = 170.0  # 180° 목표 (오차 마진 10°)
UTURN_PHASE1_TO   = 5.0    # Phase1 독립 타임아웃 (s)
UTURN_PHASE2_TO   = 5.0    # Phase2 독립 타임아웃 (s)
UTURN_STEPS = [
    ( 20,  90, 1.4),
    (-20, -90, 1.4),
    ( 20,  90, 1.4),
    (-20, -90, 1.4),
]


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
        self.avoiding    = False   # 회피 래치 활성
        self.avoid_ang   = 0       # 래치된 조향각
        self.avoid_t0    = None    # 래치 시작 시각

        self.manual_goal = None
        self.manual_done = False
        self.reversing   = False

        # IMU
        self.yaw_deg     = 0.0
        self.imu_ready   = False

        # Publisher
        self.motor_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        # Subscribers
        rospy.Subscriber('/xycar_ultrasonic',  Int32MultiArray, self.cb_sonar)
        rospy.Subscriber('/usb_cam/image_raw', Image,           self.cb_image)
        rospy.Subscriber('/rodong/vfh_cmd',    xycar_motor,     self.cb_vfh)
        rospy.Subscriber('/rodong/cmd',        String,          self.cb_cmd)
        rospy.Subscriber('/rodong/manual_goal',Point,           self.cb_manual_goal)
        rospy.Subscriber('/imu/data',          Imu,             self.cb_imu)

        rospy.loginfo('[Main] RODONG13 rev2 — IDLE. "i" 로 시작.')

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
        # quaternion → euler
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_deg  = math.degrees(euler[2])
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
            self.set_state(State.STOP)
        elif cmd == 'UTURN':
            rospy.loginfo('[Main] UTURN 명령 수신')
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

    def drive(self, speed, angle):
        msg = xycar_motor()
        msg.speed = int(speed)
        msg.angle = int(max(-90, min(90, angle)))
        self.motor_pub.publish(msg)

    def stop_motor(self):
        self.drive(0, 0)

    def reverse_motor(self, speed=20, target_front_cm=30):
        """전방 센서 기준 일정 거리까지 후진, 후방 장애물 회피 조향"""
        self.reversing = True
        rospy.loginfo('[Reverse] 후진 시작 — 전방 %.0fcm까지', target_front_cm)

        # ESC 후진 활성화 시퀀스
        self.drive(0, 0);                time.sleep(1.0)
        self.drive(-abs(speed), 0);      time.sleep(1.0)
        self.drive(0, 0);                time.sleep(1.0)

        rate    = rospy.Rate(20)
        timeout = time.time() + 8.0

        while not rospy.is_shutdown() and time.time() < timeout:
            front = min([d for d in (self.sonar[1], self.sonar[2], self.sonar[3]) if d > 0], default=999)

            # 전방이 충분히 멀어지면 종료
            if front >= target_front_cm:
                rospy.loginfo('[Reverse] 전방 %.0fcm 확보 → 완료', front)
                break

            # 후방 장애물 회피 조향
            rear_l = self.sonar[7]   # 좌후 (-135°)
            rear_r = self.sonar[5]   # 우후 (+135°)
            rear_c = self.sonar[6]   # 후방 (180°)

            if 0 < rear_c < SONAR_REVERSE:
                rospy.logwarn('[Reverse] 후방 중앙 막힘 → 정지')
                break
            elif 0 < rear_l < SONAR_REVERSE:
                steer = 90    # 좌후 막힘 → 우로 틀어서 후진
            elif 0 < rear_r < SONAR_REVERSE:
                steer = -90   # 우후 막힘 → 좌로 틀어서 후진
            else:
                steer = 0

            self.drive(-abs(speed), steer)
            rate.sleep()

        self.stop_motor()
        self.reversing = False
        rospy.loginfo('[Reverse] 후진 종료')

    def is_front_blocked(self):
        front = [self.sonar[1], self.sonar[2], self.sonar[3]]
        return any(0 < d < SONAR_EMERGENCY for d in front)

    def is_rear_blocked(self):
        rear = [self.sonar[5], self.sonar[6], self.sonar[7]]
        return any(0 < d < SONAR_REVERSE for d in rear)

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
            # Phase1: 전방 장애물은 무시 (전진+회전 중 정상)

            self.drive(SPEED_UTURN, 90)
            rate.sleep()

        self.stop_motor()
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

        self.stop_motor()

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
                if self.is_front_blocked():
                    rospy.logwarn('[UTURN/TIME] Step%d 비상정지', idx + 1)
                    self.stop_motor()
                    return
                self.drive(spd, ang)
                rate.sleep()

            self.stop_motor()
            rospy.sleep(0.2)


    # ══════════════════════════════════════════════════════════════════════
    # MANUAL DRIVE
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
            angle_deg = math.degrees(math.atan2(y, max(abs(x), 1.0)))
            angle_deg = max(-90, min(90, angle_deg))
            turn_time = abs(angle_deg) * CM_PER_DEG_TURN
            t0 = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < turn_time and not rospy.is_shutdown():
                if self.is_front_blocked():
                    self.stop_motor()
                    self.manual_done = True
                    return
                self.drive(SPEED_MANUAL, int(angle_deg))
                rate.sleep()
            self.drive(SPEED_MANUAL, 0)
            rospy.sleep(0.1)

        # Step 2: 직진
        dist = math.sqrt(x * x + y * y)
        if dist > 2.0:
            drive_time = dist / CM_PER_SEC_FWD
            spd = SPEED_MANUAL if x >= 0 else -SPEED_MANUAL
            t0 = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < drive_time and not rospy.is_shutdown():
                if self.is_front_blocked() and spd > 0:
                    self.stop_motor()
                    self.manual_done = True
                    return
                self.drive(spd, 0)
                rate.sleep()

        self.stop_motor()
        rospy.loginfo('[Manual] 도달 (추정)')
        self.manual_done = True
        self.manual_goal = None

    # ══════════════════════════════════════════════════════════════════════
    # 메인 루프
    # ══════════════════════════════════════════════════════════════════════
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.state == State.IDLE:
                self.stop_motor(); rate.sleep(); continue

            if self.state == State.STOP:
                self.stop_motor(); rate.sleep(); continue

            if self.state not in (State.UTURN, State.MANUAL_DRIVE):
                if self.reversing:
                    rate.sleep()
                    continue
                if self.is_front_blocked():
                    rospy.logwarn_throttle(1, '[Main] EMERGENCY → 후진')
                    t = threading.Thread(target=self.reverse_motor,
                                        kwargs={'speed': SPEED_NORMAL, 'target_front_cm': 30})
                    t.daemon = True
                    t.start()
                    rate.sleep()
                    continue

            if self.state == State.MANUAL_DRIVE:
                self.execute_manual_drive(); rate.sleep(); continue

            if self.state == State.UTURN:
                self.execute_uturn(); continue

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
                        self.drive(SPEED_APPROACH, int(err * gain))
                else:
                    self.drive(SPEED_APPROACH, 0)
                rate.sleep(); continue

            if self.state == State.BUG_DRIVE:
                if self.reversing:
                    rate.sleep()
                    continue
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

    def _front_min_cm(self):
        s = self.sonar
        if not s:
            return 999
        return min(s[1], s[2], s[3])

    def _bug_drive_step(self):
        now   = rospy.Time.now()
        front = self._front_min_cm()

        if not self.avoiding:
            if abs(self.vfh_angle) >= AVOID_TRIG_ANG:
                self.avoiding  = True
                self.avoid_ang = AVOID_FULL_ANG if self.vfh_angle > 0 else -AVOID_FULL_ANG
                self.avoid_t0  = now
                rospy.loginfo('[Avoid] latch %s', '우' if self.avoid_ang > 0 else '좌')

        if self.avoiding:
            self.drive(self.vfh_speed, self.avoid_ang)
            held = (now - self.avoid_t0).to_sec()
            if held >= AVOID_HOLD_MIN and front >= AVOID_CLEAR_CM:
                self.avoiding = False
                rospy.loginfo('[Avoid] release (front=%dcm)', front)
        else:
            self.drive(self.vfh_speed, self.vfh_angle)

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
                    rospy.loginfo_throttle(0.5, "[ArUco] marker_w=%dpx  marker_cx=%dpx", self.marker_w, self.marker_cx)
                    return True
        return False


if __name__ == '__main__':
    try:
        RodongMain().run()
    except rospy.ROSInterruptException:
        pass
