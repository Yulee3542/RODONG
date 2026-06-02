#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
vfh_planner.py  (RODONG13 - 센서 융합 능동 회피)
================================================================
역할: RODONG 회피 두뇌. 초음파 + ArUco goal + YOLO + 바닥경계 판단을 융합해
      VFH+ 알고리즘으로 (speed, angle)을 계산하여 /rodong/vfh_cmd 발행.

구독:
  /xycar_ultrasonic  (std_msgs/Int32MultiArray, 8빔)
  /aruco_pose        (geometry_msgs/PoseStamped)      - VFH+ goal
  /rodong/yolo       (std_msgs/Float32MultiArray)     - 카메라 판단 (선택)
                      data = [class_id, conf, cx_norm, cy_norm, bottom_y_ratio]
  /rodong/boundary   (std_msgs/Float32MultiArray)     - 바닥 경계선 (선택)
                      data = [left, center, right, near]

발행:
  /rodong/vfh_cmd    (xycar_msgs/xycar_motor)

설계 노트:
  - 상수/초음파빔/히스토그램 로직은 rodong_config / rodong_sonar 로 분리(단일 출처).
  - rodong_main.py 는 이 토픽을 구독해 BUG_DRIVE 상태일 때만 /xycar_motor 로 전달.
  - goal(ArUco) 가 보이면 그 방향을 선호, 없으면 직진(섹터3)을 목표로 회피 주행.
  - YOLO AVOID / 바닥경계 는 해당 방향 섹터에 가상 장애물을 추가(능동 회피).
"""

import os
import sys
import math
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from xycar_msgs.msg import xycar_motor

# catkin 은 devel/lib 래퍼에서 노드를 실행하므로 scripts/ 가 sys.path 에 없음.
# 동일 폴더의 공용 모듈(rodong_config 등)을 import 하도록 경로 추가.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rodong_config as cfg
from rodong_geometry import angle_to_sector, clip
from rodong_sonar import build_histogram, select_sector, front_min


class VFHPlanner:
    def __init__(self):
        rospy.init_node('vfh_planner', anonymous=False)

        self.sonar = [999] * 8

        # ArUco goal
        self.goal_bearing  = 0.0    # [rad], 좌(-)/우(+)
        self.goal_distance = None   # [m]
        self.last_goal_t   = None

        # YOLO 판단
        self.yolo_cls    = -1
        self.yolo_cx     = 0.0      # -1.0 ~ 1.0
        self.yolo_bottom = 0.0
        self.last_yolo_t = None

        # 바닥 경계선 [left, center, right, near] 검은픽셀 비율
        self.bound        = [0.0, 0.0, 0.0, 0.0]
        self.last_bound_t = None

        self.prev_steer = 0.0       # 직전 선택 조향각 [deg] (smooth용)

        self.pub = rospy.Publisher('/rodong/vfh_cmd', xycar_motor, queue_size=1)

        rospy.Subscriber('/xycar_ultrasonic', Int32MultiArray,
                         self.cb_sonar, queue_size=1)
        rospy.Subscriber('/aruco_pose', PoseStamped,
                         self.cb_goal, queue_size=1)
        rospy.Subscriber('/rodong/yolo', Float32MultiArray,
                         self.cb_yolo, queue_size=1)
        rospy.Subscriber('/rodong/boundary', Float32MultiArray,
                         self.cb_boundary, queue_size=1)

        rospy.loginfo("[VFH+] sensor-fusion planner started (threshold=%.0fcm)",
                      cfg.THRESHOLD)
        rospy.Timer(rospy.Duration(0.1), self.cb_timer)   # 10Hz

    # ── 콜백 ─────────────────────────────────────────────────────
    def cb_sonar(self, msg):
        self.sonar = list(msg.data[:8])

    def cb_goal(self, msg):
        self.goal_bearing  = msg.pose.orientation.z   # [rad]
        self.goal_distance = msg.pose.position.x      # [m]
        self.last_goal_t   = rospy.Time.now()

    def cb_yolo(self, msg):
        d = list(msg.data)
        if len(d) >= 5:
            self.yolo_cls    = int(d[0])
            self.yolo_cx     = d[2]
            self.yolo_bottom = d[4]
            self.last_yolo_t = rospy.Time.now()

    def cb_boundary(self, msg):
        d = list(msg.data)
        if len(d) >= 4:
            self.bound        = d[:4]
            self.last_bound_t = rospy.Time.now()

    # ── 유효성 체크 ──────────────────────────────────────────────
    def _valid(self, t, timeout):
        return t is not None and (rospy.Time.now() - t).to_sec() < timeout

    # ── 메인 루프 (10Hz) ─────────────────────────────────────────
    def cb_timer(self, event):
        goal_valid = self._valid(self.last_goal_t, cfg.GOAL_TIMEOUT)
        yolo_valid = self._valid(self.last_yolo_t, cfg.YOLO_TIMEOUT)

        # ── YOLO 의도 해석 ───────────────────────────────────────
        # climb_now: 정면 물체를 넘어갈 대상으로 보고 장애물 판정 완화
        # avoid_dir: 회피해야 할 화면상 방향(좌/우) → 가상 장애물 추가
        climb_now = False
        avoid_dir = None   # 'left' / 'right' / 'center'
        if yolo_valid:
            if (cfg.USE_CLIMB and self.yolo_cls == cfg.CLS_CLIMB
                    and self.yolo_bottom >= cfg.CLIMB_BOTTOM_RATIO):
                climb_now = True
            elif self.yolo_cls == cfg.CLS_AVOID:
                if self.yolo_cx < -0.15:
                    avoid_dir = 'left'
                elif self.yolo_cx > 0.15:
                    avoid_dir = 'right'
                else:
                    avoid_dir = 'center'

        # ── 1. Polar Histogram (초음파 7섹터) ────────────────────
        hist = build_histogram(self.sonar, climb_now=climb_now)

        # ── YOLO AVOID → 가상 장애물 추가 ────────────────────────
        if avoid_dir == 'left':
            hist[0] += 1.0; hist[1] += 1.0; hist[2] += 0.7
        elif avoid_dir == 'right':
            hist[6] += 1.0; hist[5] += 1.0; hist[4] += 0.7
        elif avoid_dir == 'center':
            hist[2] += 0.8; hist[3] += 1.0; hist[4] += 0.8

        # ── 바닥 검은 경계선 → 가상 장애물 (경계 밖 이탈 방지) ────
        # 경계가 보이는 쪽 섹터를 막아 반대(안쪽)로 조향하게 만든다.
        if self._valid(self.last_bound_t, cfg.BOUND_TIMEOUT):
            bl, bc, br, bn = self.bound
            if bl > cfg.BOUND_TH:              # 좌측 경계 → 좌측 차단 → 우조향 유도
                hist[0] += 1.5; hist[1] += 1.2; hist[2] += 0.6
            if br > cfg.BOUND_TH:              # 우측 경계 → 우측 차단 → 좌조향 유도
                hist[6] += 1.5; hist[5] += 1.2; hist[4] += 0.6
            if bc > cfg.BOUND_TH:              # 정면 경계
                hist[2] += 0.8; hist[3] += 1.2; hist[4] += 0.8
            if bn > cfg.BOUND_NEAR_TH:         # 차 바로 앞 경계 임박 → 정면 강하게 차단
                hist[2] += 1.5; hist[3] += 2.0; hist[4] += 1.5
            rospy.loginfo_throttle(2.0,
                "[VFH+] boundary L=%.2f C=%.2f R=%.2f near=%.2f",
                bl, bc, br, bn)

        # ── 2. 목표 섹터 결정 ────────────────────────────────────
        goal_deg = math.degrees(self.goal_bearing) if goal_valid else 0.0
        goal_sector = angle_to_sector(goal_deg)
        if goal_sector is None:
            goal_sector = 3

        # ── 3. 후보 섹터 비용 평가 ───────────────────────────────
        best_sector = select_sector(hist, goal_sector, self.prev_steer)

        # ── 4. 모터 명령 ─────────────────────────────────────────
        if best_sector is None:
            # 전방향 막힘 → 후진
            rospy.logwarn_throttle(1.0, "[VFH+] all blocked → reverse")
            self.prev_steer = 0.0
            self.publish(cfg.SPEED_BACK, 0)
            return

        steer = int(clip(cfg.SECTOR_ANGLE[best_sector], -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
        self.prev_steer = steer

        # 속도: 정면 가까우면 감속
        min_front = front_min(self.sonar)
        speed = cfg.SPEED_DRIVE
        if climb_now:
            speed = cfg.SPEED_DRIVE    # 등반 대상 접근 시 감속 (현재 동일값)

        self.publish(speed, steer)

        rospy.loginfo_throttle(5.0,
            "[VFH+] goal=%.0fdeg sec=%d steer=%d spd=%d front=%.0fcm "
            "climb=%s avoid=%s",
            goal_deg, best_sector, steer, speed, min_front,
            climb_now, avoid_dir)

    def publish(self, speed, angle):
        msg = xycar_motor()
        msg.speed = int(speed)
        msg.angle = int(angle)
        self.pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        VFHPlanner().run()
    except rospy.ROSInterruptException:
        pass
