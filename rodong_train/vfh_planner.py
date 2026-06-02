#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
vfh_planner.py  (RODONG12 - 센서 융합 능동 회피)
================================================================
역할: RODONG 회피 두뇌. 초음파 + ArUco goal + YOLO 판단을 융합해
      VFH+ 알고리즘으로 (speed, angle)을 계산하여 /rodong/vfh_cmd 발행.

구독:
  /xycar_ultrasonic  (std_msgs/Int32MultiArray, 8빔)
  /aruco_pose        (geometry_msgs/PoseStamped)      - VFH+ goal
  /rodong/yolo       (std_msgs/Float32MultiArray)     - 카메라 판단 (선택)
                      data = [class_id, conf, cx_norm, cy_norm, bottom_y_ratio]
                      class_id: 0=CLIMB 1=AVOID 2=IGNORE -1=없음

발행:
  /rodong/vfh_cmd    (xycar_msgs/xycar_motor)

설계 노트:
  - rodong_main.py 는 이 토픽을 구독해서 BUG_DRIVE 상태일 때만 /xycar_motor 로 전달.
  - goal(ArUco) 가 보이면 그 방향을 선호, 없으면 직진(섹터3)을 목표로 회피 주행.
  - YOLO 가 AVOID 를 보고하면 해당 방향 섹터에 가상 장애물을 추가(능동 회피).
  - YOLO 가 CLIMB 를 보고하면 정면 장애물을 무시(넘어갈 대상)하고 직진 유지.
"""

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from xycar_msgs.msg import xycar_motor

# ── VFH 히스토그램 파라미터 ──────────────────────────────────────
# 전방 ±90도만 다루는 7섹터 (각 30도). 후방 빔은 회피 계산에서 제외.
#   섹터:  0     1     2    3   4    5    6
#   각도: -90   -60   -30   0  +30  +60  +90
N_SECTORS  = 7
SECTOR_DEG = 30.0
SECTOR_ANGLE = [-90, -60, -30, 0, 30, 60, 90]   # 섹터 중심각 → 조향각

# ── 거리/임계값 [cm] ─────────────────────────────────────────────
THRESHOLD      = 30.0   # 이 거리 이하 → 장애물로 간주 (실주행 기본값)
THRESHOLD_DESK = 5.0    # 책상 테스트용 (필요시 THRESHOLD 교체)
SLOW_DIST      = 50.0   # 이 거리 이하 → 감속
EMERGENCY      = 12.0   # 이 거리 이하(정면) → 후진/정지 신호

# ── 모터 파라미터 ────────────────────────────────────────────────
SPEED_NORMAL = 30
SPEED_SLOW   = 25
SPEED_BACK   = -20      # 전방향 막힘 시 후진
ANGLE_MAX    = 90       # 조향 최대 (실측: ±90 정상)

# ── VFH 비용 가중치 ──────────────────────────────────────────────
W_GOAL    = 1.0   # goal 방향에 가까울수록 선호
W_HEADING = 0.4   # 현재 진행방향 유지 (지그재그 방지)
W_SMOOTH  = 0.2   # 직전 선택 방향 유지 (떨림 방지)

# ── 타임아웃 [sec] ───────────────────────────────────────────────
GOAL_TIMEOUT = 1.5
YOLO_TIMEOUT = 1.0

# ── 초음파 빔 각도 [deg] (메모리 매핑) ───────────────────────────
# idx0=좌(-90) 1=좌전(-45) 2=우전(+45) 3=전(0)
# idx4=우(+90) 5=우후(+135) 6=후(180) 7=좌후(-135)
BEAM_ANGLES = [-90, -45, 45, 0, 90, 135, 180, -135]

# ── YOLO 클래스 ──────────────────────────────────────────────────
CLS_CLIMB  = 0
CLS_AVOID  = 1
CLS_IGNORE = 2
CLIMB_BOTTOM_RATIO = 0.82   # 메모리: bottom_y/frame_h >= 0.82 → CLIMB 후보

# ── YOLO 사용 정책 ───────────────────────────────────────────────
# 현재 모델 mAP50=0.30 으로 신뢰도 낮음. CLIMB 판정은 끄고 AVOID 보조만 사용.
# 모델 성능이 충분해지면 USE_CLIMB=True 로 등반 회피 완화 로직 활성화.
USE_CLIMB = False


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

        self.prev_steer = 0.0       # 직전 선택 조향각 [deg] (smooth용)

        self.pub = rospy.Publisher('/rodong/vfh_cmd', xycar_motor, queue_size=1)

        rospy.Subscriber('/xycar_ultrasonic', Int32MultiArray,
                         self.cb_sonar, queue_size=1)
        rospy.Subscriber('/aruco_pose', PoseStamped,
                         self.cb_goal, queue_size=1)
        rospy.Subscriber('/rodong/yolo', Float32MultiArray,
                         self.cb_yolo, queue_size=1)

        rospy.loginfo("[VFH+] sensor-fusion planner started (threshold=%.0fcm)",
                      THRESHOLD)
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

    # ── 유효성 체크 ──────────────────────────────────────────────
    def _valid(self, t, timeout):
        return t is not None and (rospy.Time.now() - t).to_sec() < timeout

    # ── 각도 → 섹터 인덱스 ───────────────────────────────────────
    @staticmethod
    def _angle_to_sector(deg):
        # -90~+90 deg → 0~6. 범위 밖이면 None.
        if deg < -105 or deg > 105:
            return None
        s = int(round((deg + 90) / SECTOR_DEG))
        return max(0, min(N_SECTORS - 1, s))

    # ── 메인 루프 (10Hz) ─────────────────────────────────────────
    def cb_timer(self, event):
        goal_valid = self._valid(self.last_goal_t, GOAL_TIMEOUT)
        yolo_valid = self._valid(self.last_yolo_t, YOLO_TIMEOUT)

        # ── YOLO 의도 해석 ───────────────────────────────────────
        # climb_now: 정면 물체를 넘어갈 대상으로 보고 장애물 판정 완화
        # avoid_dir: 회피해야 할 화면상 방향(좌/우) → 가상 장애물 추가
        climb_now = False
        avoid_dir = None   # 'left' / 'right' / 'center'
        if yolo_valid:
            if (USE_CLIMB and self.yolo_cls == CLS_CLIMB
                    and self.yolo_bottom >= CLIMB_BOTTOM_RATIO):
                climb_now = True
            elif self.yolo_cls == CLS_AVOID:
                if self.yolo_cx < -0.15:
                    avoid_dir = 'left'
                elif self.yolo_cx > 0.15:
                    avoid_dir = 'right'
                else:
                    avoid_dir = 'center'

        # ── 1. Polar Histogram (7섹터) ───────────────────────────
        hist = np.zeros(N_SECTORS)

        for i, ang in enumerate(BEAM_ANGLES):
            s = self._angle_to_sector(ang)
            if s is None:
                continue                      # 후방 빔 무시
            dist = self.sonar[i]
            if dist <= 0:
                continue
            if dist < THRESHOLD:
                # CLIMB 대상이면 정면 장애물 영향 완화
                if climb_now and abs(ang) <= 45:
                    continue
                weight = (THRESHOLD - dist) / THRESHOLD
                hist[s] += weight
                if s > 0:
                    hist[s - 1] += weight * 0.5    # 인접 섹터 번짐
                if s < N_SECTORS - 1:
                    hist[s + 1] += weight * 0.5

        # ── YOLO AVOID → 가상 장애물 추가 ────────────────────────
        if avoid_dir == 'left':
            hist[0] += 1.0; hist[1] += 1.0; hist[2] += 0.7
        elif avoid_dir == 'right':
            hist[6] += 1.0; hist[5] += 1.0; hist[4] += 0.7
        elif avoid_dir == 'center':
            hist[2] += 0.8; hist[3] += 1.0; hist[4] += 0.8

        # ── 2. 목표 섹터 결정 ────────────────────────────────────
        if goal_valid:
            goal_deg = np.degrees(self.goal_bearing)
        else:
            goal_deg = 0.0    # goal 없으면 직진 목표
        goal_sector = self._angle_to_sector(goal_deg)
        if goal_sector is None:
            goal_sector = 3

        # ── 3. 후보 섹터 비용 평가 ───────────────────────────────
        OPEN_THRESH = 0.5     # hist 이하면 통행 가능 섹터로 간주
        best_sector, best_cost = None, float('inf')
        for s in range(N_SECTORS):
            if hist[s] > OPEN_THRESH:
                continue
            diff_goal    = abs(s - goal_sector)
            diff_heading = abs(SECTOR_ANGLE[s] - 0)     # 직진 대비
            diff_smooth  = abs(SECTOR_ANGLE[s] - self.prev_steer)
            cost = (W_GOAL * diff_goal +
                    W_HEADING * (diff_heading / 30.0) +
                    W_SMOOTH * (diff_smooth / 30.0))
            if cost < best_cost:
                best_cost, best_sector = cost, s

        # ── 4. 모터 명령 ─────────────────────────────────────────
        if best_sector is None:
            # 전방향 막힘 → 후진
            rospy.logwarn_throttle(1.0, "[VFH+] all blocked → reverse")
            self.prev_steer = 0.0
            self.publish(SPEED_BACK, 0)
            return

        steer = SECTOR_ANGLE[best_sector]
        steer = int(np.clip(steer, -ANGLE_MAX, ANGLE_MAX))
        self.prev_steer = steer

        # 속도: 정면 가까우면 감속
        valid_front = [d for d in (self.sonar[1], self.sonar[2], self.sonar[3])
                       if d > 0]
        min_front = min(valid_front) if valid_front else 999
        speed = SPEED_SLOW if min_front < SLOW_DIST else SPEED_NORMAL
        if climb_now:
            speed = SPEED_SLOW    # 등반 대상 접근 시 감속

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
