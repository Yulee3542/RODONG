#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rodong_main.py  —  RODONG13
변경사항:
  - IDLE 상태 추가: launch 후 init 명령 대기
  - MANUAL_DRIVE 상태 추가: dead reckoning 좌표 이동
  - /rodong/cmd 구독 (String: INIT|STOP|MANUAL)
  - /rodong/manual_goal 구독 (geometry_msgs/Point, cm 단위)
"""

import rospy
import math
import time
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from xycar_msgs.msg import xycar_motor
import cv2
from cv_bridge import CvBridge
import numpy as np

# ── ArUco (Pi: OpenCV 4.5.x old API) ──────────────────────────────────────
ARUCO_DICT   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
TARGET_ID    = 1

# ── 상태 정의 ──────────────────────────────────────────────────────────────
class State:
    IDLE            = 'IDLE'            # launch 후 대기 (init 명령 필요)
    BUG_DRIVE       = 'BUG_DRIVE'       # 자율주행 (VFH+)
    MARKER_APPROACH = 'MARKER_APPROACH' # ArUco 마커 접근
    UTURN           = 'UTURN'           # 3점 턴
    MANUAL_DRIVE    = 'MANUAL_DRIVE'    # 좌표 지정 이동
    STOP            = 'STOP'            # 긴급 정지

# ── 파라미터 ───────────────────────────────────────────────────────────────
SPEED_NORMAL   = 30
SPEED_APPROACH = 25
SPEED_UTURN    = 20
SPEED_MANUAL   = 28

ANGLE_CENTER   = 0
ANGLE_UTURN    = 80

SONAR_EMERGENCY  = 5    # cm: 이 거리 미만이면 즉시 정지
MARKER_CLOSE_PX  = 80   # 마커 너비(px) 이상이면 접근 완료
MARKER_DEBOUNCE  = 5    # 마커 연속 감지 프레임 수

# dead reckoning 속도 상수 (speed=30 기준 실험값, 나중에 캘리브레이션)
# cm/s — 실제 측정 후 수정 필요
CM_PER_SEC_FWD  = 18.0   # speed=SPEED_MANUAL 기준 직진 속도
CM_PER_DEG_TURN = 0.12   # speed=SPEED_MANUAL, angle=90 기준 (시간 기반)

# ── UTURN 시퀀스 ───────────────────────────────────────────────────────────
UTURN_STEPS = [
    ( SPEED_UTURN,  ANGLE_UTURN, 1.2),   # 전진+우회전
    (-SPEED_UTURN, -ANGLE_UTURN, 1.2),   # 후진+좌회전
    (-SPEED_UTURN, -ANGLE_UTURN, 1.2),   # 후진+좌회전 반복
]


class RodongMain:
    def __init__(self):
        rospy.init_node('rodong_main', anonymous=False)

        # 상태
        self.state          = State.IDLE
        self.prev_state     = None

        # 센서
        self.sonar          = [999] * 8
        self.bridge         = CvBridge()
        self.frame          = None

        # 마커 디바운스
        self.marker_seen    = 0
        self.marker_cx      = -1
        self.marker_w       = 0

        # VFH cmd (vfh_planner 로부터)
        self.vfh_speed      = 0
        self.vfh_angle      = 0

        # manual drive
        self.manual_goal    = None   # Point (cm)
        self.manual_done    = False

        # Publisher
        self.motor_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        # Subscribers
        rospy.Subscriber('/xycar_ultrasonic',  Int32MultiArray, self.cb_sonar)
        rospy.Subscriber('/usb_cam/image_raw', Image,           self.cb_image)
        rospy.Subscriber('/rodong/vfh_cmd',    xycar_motor,     self.cb_vfh)
        rospy.Subscriber('/rodong/cmd',        String,          self.cb_cmd)
        rospy.Subscriber('/rodong/manual_goal',Point,           self.cb_manual_goal)

        rospy.loginfo('[Main] RODONG13 initialized — IDLE 상태. "i" 키로 자율주행 시작.')

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def cb_sonar(self, msg):
        self.sonar = list(msg.data)

    def cb_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5, '[Main] Image convert error: %s', e)

    def cb_vfh(self, msg):
        self.vfh_speed = msg.speed
        self.vfh_angle = msg.angle

    def cb_cmd(self, msg):
        cmd = msg.data.strip().upper()
        rospy.loginfo('[Main] CMD received: %s', cmd)

        if cmd == 'INIT':
            if self.state in (State.IDLE, State.STOP):
                self.set_state(State.BUG_DRIVE)
            elif self.state == State.MANUAL_DRIVE:
                # manual 도중 INIT → 자율주행 복귀
                self.manual_goal = None
                self.set_state(State.BUG_DRIVE)

        elif cmd == 'STOP':
            self.set_state(State.STOP)

        elif cmd == 'MANUAL':
            # teleop이 MANUAL 모드 진입을 알려줌 → 대기 상태로
            # 실제 이동은 manual_goal 수신 시 시작
            rospy.loginfo('[Main] MANUAL 모드 대기 중 — 좌표를 입력하세요.')
            self.set_state(State.MANUAL_DRIVE)
            self.manual_goal = None
            self.manual_done = True  # 목표 없음 = 제자리 대기

    def cb_manual_goal(self, msg):
        if self.state == State.MANUAL_DRIVE:
            rospy.loginfo('[Main] 새 목표: x=%.1f  y=%.1f cm', msg.x, msg.y)
            self.manual_goal = msg
            self.manual_done = False

    # ── 상태 전환 ─────────────────────────────────────────────────────────
    def set_state(self, new_state):
        if new_state != self.state:
            rospy.loginfo('[Main] 상태 전환: %s → %s', self.state, new_state)
            self.prev_state = self.state
            self.state = new_state

    # ── 모터 ──────────────────────────────────────────────────────────────
    def drive(self, speed, angle):
        msg = xycar_motor()
        msg.speed = int(speed)
        msg.angle = int(max(-90, min(90, angle)))
        self.motor_pub.publish(msg)

    def stop_motor(self):
        self.drive(0, 0)

    # ── 긴급 정지 체크 ─────────────────────────────────────────────────────
    def is_emergency(self):
        front = [self.sonar[1], self.sonar[2], self.sonar[3]]
        return any(0 < d < SONAR_EMERGENCY for d in front)

    # ── ArUco 감지 ────────────────────────────────────────────────────────
    def detect_marker(self):
        if self.frame is None:
            return False
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if mid == TARGET_ID:
                    pts = corners[i][0]
                    cx = int(pts[:, 0].mean())
                    w  = int(pts[:, 0].max() - pts[:, 0].min())
                    self.marker_cx = cx
                    self.marker_w  = w
                    return True
        return False

    # ══════════════════════════════════════════════════════════════════════
    # MANUAL DRIVE — dead reckoning 좌표 이동
    # 좌표계: x=전방(cm), y=좌측(cm)  (오른쪽이 -y)
    # 전략: (1) y축 회전 정렬 → (2) x축 직진
    # ══════════════════════════════════════════════════════════════════════
    def execute_manual_drive(self):
        goal = self.manual_goal
        if goal is None or self.manual_done:
            self.stop_motor()
            return

        x = goal.x   # 전방 cm
        y = goal.y   # 좌측 cm (양수=좌, 음수=우)

        rospy.loginfo('[Manual] 목표: x=%.1f  y=%.1f cm', x, y)

        # ── Step 1: y축 정렬 (좌우 회전) ────────────────────────────────
        if abs(y) > 2.0:
            angle_deg = math.degrees(math.atan2(y, max(abs(x), 1.0)))
            angle_deg = max(-90, min(90, angle_deg))
            # 회전에 필요한 시간 추정
            turn_time = abs(angle_deg) * CM_PER_DEG_TURN
            rospy.loginfo('[Manual] Step1 회전: %.1f°  %.2fs', angle_deg, turn_time)
            t0 = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < turn_time and not rospy.is_shutdown():
                if self.is_emergency():
                    rospy.logwarn('[Manual] 긴급 정지!')
                    self.stop_motor()
                    self.manual_done = True
                    return
                self.drive(SPEED_MANUAL, int(angle_deg))
                rate.sleep()
            # 회전 후 직진 준비
            self.drive(SPEED_MANUAL, 0)
            rospy.sleep(0.1)

        # ── Step 2: 직진 (x + y 합성 거리) ─────────────────────────────
        dist = math.sqrt(x * x + y * y)
        if dist > 2.0:
            drive_time = dist / CM_PER_SEC_FWD
            if x < 0:
                drive_time = -drive_time  # 후진
            rospy.loginfo('[Manual] Step2 직진: %.1f cm  %.2fs', dist, abs(drive_time))
            spd = SPEED_MANUAL if drive_time > 0 else -SPEED_MANUAL
            t0 = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < abs(drive_time) and not rospy.is_shutdown():
                if self.is_emergency() and spd > 0:
                    rospy.logwarn('[Manual] 긴급 정지!')
                    self.stop_motor()
                    self.manual_done = True
                    return
                self.drive(spd, 0)
                rate.sleep()

        self.stop_motor()
        rospy.loginfo('[Manual] 목표 도달 (추정)')
        self.manual_done = True
        self.manual_goal = None

    # ══════════════════════════════════════════════════════════════════════
    # UTURN (3점 턴)
    # ══════════════════════════════════════════════════════════════════════
    def execute_uturn(self):
        rospy.loginfo('[Main] U-TURN 시작')
        for spd, ang, dur in UTURN_STEPS:
            if rospy.is_shutdown():
                break
            t0 = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < dur and not rospy.is_shutdown():
                self.drive(spd, ang)
                rate.sleep()
        self.stop_motor()
        rospy.loginfo('[Main] U-TURN 완료')
        self.set_state(State.BUG_DRIVE)

    # ══════════════════════════════════════════════════════════════════════
    # 메인 루프
    # ══════════════════════════════════════════════════════════════════════
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            # ── IDLE: 명령 대기 ────────────────────────────────────────
            if self.state == State.IDLE:
                self.stop_motor()
                rate.sleep()
                continue

            # ── STOP ──────────────────────────────────────────────────
            if self.state == State.STOP:
                self.stop_motor()
                rate.sleep()
                continue

            # ── 긴급 정지 (MANUAL 중 후진은 제외) ─────────────────────
            if self.state not in (State.UTURN, State.MANUAL_DRIVE):
                if self.is_emergency():
                    rospy.logwarn_throttle(1, '[Main] SONAR EMERGENCY → STOP')
                    self.stop_motor()
                    rate.sleep()
                    continue

            # ── MANUAL_DRIVE ──────────────────────────────────────────
            if self.state == State.MANUAL_DRIVE:
                self.execute_manual_drive()
                rate.sleep()
                continue

            # ── UTURN ─────────────────────────────────────────────────
            if self.state == State.UTURN:
                self.execute_uturn()
                continue

            # ── MARKER_APPROACH ───────────────────────────────────────
            if self.state == State.MARKER_APPROACH:
                if self.detect_marker():
                    if self.marker_w >= MARKER_CLOSE_PX:
                        rospy.loginfo('[Main] 마커 접근 완료 → UTURN')
                        self.stop_motor()
                        self.set_state(State.UTURN)
                    else:
                        # 마커 중앙 정렬하며 접근
                        frame_w = self.frame.shape[1] if self.frame is not None else 640
                        err = self.marker_cx - frame_w // 2
                        gain = 150.0 / (frame_w // 2)
                        steer = int(err * gain)
                        self.drive(SPEED_APPROACH, steer)
                else:
                    # 마커 놓침 → 직진 유지
                    self.drive(SPEED_APPROACH, 0)
                rate.sleep()
                continue

            # ── BUG_DRIVE (VFH+ 자율주행) ─────────────────────────────
            if self.state == State.BUG_DRIVE:
                # 마커 감지 디바운스
                if self.detect_marker():
                    self.marker_seen += 1
                else:
                    self.marker_seen = 0

                if self.marker_seen >= MARKER_DEBOUNCE:
                    rospy.loginfo('[Main] 마커 감지 → MARKER_APPROACH')
                    self.marker_seen = 0
                    self.set_state(State.MARKER_APPROACH)
                    continue

                # VFH 명령 그대로 전달
                self.drive(self.vfh_speed, self.vfh_angle)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = RodongMain()
        node.run()
    except rospy.ROSInterruptException:
        pass
