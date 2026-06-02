#!/usr/bin/env python3
"""
RODONG - Obstacle Avoidance Node (ROS1 Noetic)

센서 fusion:
  - USB 카메라 → HSV 마스킹 기반 전방 장애물 영역 검출
  - HC-SR04 초음파(N채널) → 좌/우/전방 거리로 분류, 회피 방향 결정

publish:   /xycar_motor  (xycar_msgs/xycar_motor)
subscribe:
  /camera/image_raw   (sensor_msgs/Image)
  /ultrasonic         (std_msgs/Int32MultiArray  →  [front, left, right, ...])
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge

import cv2
import numpy as np


# ── 튜닝 파라미터 ─────────────────────────────────────────────────────────────
ULTRASONIC_FRONT_THRESHOLD = 30   # cm: 전방 이 거리 이하면 장애물
ULTRASONIC_SIDE_THRESHOLD  = 20   # cm: 측면 이 거리 이하면 벽에 가까움

SPEED_NORMAL  =  25   # 직진 속도  (-50 ~ 50)
SPEED_SLOW    =  12   # 회피 중 속도
SPEED_STOP    =   0   # 정지

ANGLE_STRAIGHT =  0   # 조향 중립
ANGLE_LEFT     = -30  # 좌회전
ANGLE_RIGHT    =  30  # 우회전
ANGLE_HARD_L   = -50  # 급좌회전
ANGLE_HARD_R   =  50  # 급우회전

# 카메라 장애물 검출: 하단 ROI 비율
ROI_TOP_RATIO         = 0.55
OBSTACLE_PIX_THRESHOLD = 3000

# HSV 범위: 붉은색/주황색 계열 장애물 콘
OBSTACLE_HSV_LOWER1 = np.array([0,   80,  80])
OBSTACLE_HSV_UPPER1 = np.array([15, 255, 255])
OBSTACLE_HSV_LOWER2 = np.array([160, 80,  80])
OBSTACLE_HSV_UPPER2 = np.array([180, 255, 255])

AVOID_TICKS = 30   # 30 tick × 0.05s = 1.5초 회피 유지
# ─────────────────────────────────────────────────────────────────────────────


class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=False)
        rospy.loginfo('RODONG obstacle avoidance node started')

        self.bridge = CvBridge()
        self.latest_image = None
        self.ultrasonic = []

        self.cam_obstacle_detected = False
        self.cam_obstacle_side = 'center'

        self.state = 'forward'
        self.avoid_counter = 0

        # ── Subscribers ──────────────────────────────────────────────────────
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber('/ultrasonic', Int32MultiArray, self.ultrasonic_callback,
                         queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_motor = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        # ── 제어 루프 20Hz ────────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f'CV bridge error: {e}')
            return
        self.latest_image = frame
        self._process_camera(frame)

    def ultrasonic_callback(self, msg):
        self.ultrasonic = list(msg.data)

    # ── 카메라 처리 ────────────────────────────────────────────────────────────

    def _process_camera(self, frame):
        h, w = frame.shape[:2]
        roi_top = int(h * ROI_TOP_RATIO)
        roi = frame[roi_top:h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, OBSTACLE_HSV_LOWER1, OBSTACLE_HSV_UPPER1)
        mask2 = cv2.inRange(hsv, OBSTACLE_HSV_LOWER2, OBSTACLE_HSV_UPPER2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        total_pixels = np.sum(mask > 0)

        if total_pixels > OBSTACLE_PIX_THRESHOLD:
            self.cam_obstacle_detected = True
            left_px  = np.sum(mask[:, :w//2] > 0)
            right_px = np.sum(mask[:, w//2:] > 0)
            if left_px > right_px * 1.3:
                self.cam_obstacle_side = 'left'
            elif right_px > left_px * 1.3:
                self.cam_obstacle_side = 'right'
            else:
                self.cam_obstacle_side = 'center'
        else:
            self.cam_obstacle_detected = False
            self.cam_obstacle_side = 'center'

        if self.cam_obstacle_detected:
            rospy.logdebug(f'Camera obstacle: {self.cam_obstacle_side}, px={total_pixels}')

    # ── 초음파 파싱 ────────────────────────────────────────────────────────────

    def _get_ultrasonic(self):
        if not self.ultrasonic:
            return 9999, 9999, 9999
        data = self.ultrasonic
        front = data[0] if len(data) > 0 else 9999
        left  = data[1] if len(data) > 1 else 9999
        right = data[2] if len(data) > 2 else 9999
        front = front if front > 0 else 9999
        left  = left  if left  > 0 else 9999
        right = right if right > 0 else 9999
        return front, left, right

    # ── 메인 제어 루프 ─────────────────────────────────────────────────────────

    def control_loop(self, event=None):
        front, left, right = self._get_ultrasonic()

        speed = SPEED_NORMAL
        angle = ANGLE_STRAIGHT

        if self.state == 'forward':
            if front < ULTRASONIC_FRONT_THRESHOLD:
                rospy.loginfo(f'Ultrasonic obstacle! front={front}cm')
                self.state = 'avoid_right' if right > left else 'avoid_left'
                self.avoid_counter = AVOID_TICKS

            elif self.cam_obstacle_detected:
                rospy.loginfo(f'Camera obstacle! side={self.cam_obstacle_side}')
                if self.cam_obstacle_side == 'left':
                    self.state = 'avoid_right'
                elif self.cam_obstacle_side == 'right':
                    self.state = 'avoid_left'
                else:
                    self.state = 'avoid_right' if right >= left else 'avoid_left'
                self.avoid_counter = AVOID_TICKS

            else:
                speed = SPEED_NORMAL
                angle = ANGLE_STRAIGHT

        elif self.state == 'avoid_left':
            speed = SPEED_SLOW
            angle = ANGLE_HARD_L
            self.avoid_counter -= 1
            if self.avoid_counter <= 0:
                self.state = 'forward'
                rospy.loginfo('Avoidance complete → forward')

        elif self.state == 'avoid_right':
            speed = SPEED_SLOW
            angle = ANGLE_HARD_R
            self.avoid_counter -= 1
            if self.avoid_counter <= 0:
                self.state = 'forward'
                rospy.loginfo('Avoidance complete → forward')

        elif self.state == 'stop':
            speed = SPEED_STOP
            angle = ANGLE_STRAIGHT

        # 측면 초음파 미세 조향 보정 (forward 상태에서만)
        if self.state == 'forward':
            if left < ULTRASONIC_SIDE_THRESHOLD:
                angle = max(angle, ANGLE_RIGHT // 2)
            elif right < ULTRASONIC_SIDE_THRESHOLD:
                angle = min(angle, ANGLE_LEFT // 2)

        self._publish_motor(speed, angle)

    # ── 퍼블리시 ──────────────────────────────────────────────────────────────

    def _publish_motor(self, speed, angle):
        msg = xycar_motor()
        msg.speed = speed
        msg.angle = angle
        self.pub_motor.publish(msg)


def main():
    node = ObstacleAvoidanceNode()
    rospy.spin()


if __name__ == '__main__':
    main()
