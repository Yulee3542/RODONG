#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
aruco_detector.py
- /usb_cam/image_raw 구독
- ArUco ID=1 검출 (Old API, OpenCV 4.5)
- 픽셀 기반 거리/방향 추정 → /aruco_pose (PoseStamped) publish
- 미검출 시 /aruco_pose publish 안 함
"""

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

# ── 카메라 파라미터 (캘리브레이션 전 추정값) ──────────────────────
IMG_W = 640
IMG_H = 480
FOCAL_LEN = 600.0          # 픽셀 단위 추정 초점거리
MARKER_SIZE_M = 0.15       # 마커 실제 크기 [m]

TARGET_ID = 1
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
PARAMS     = aruco.DetectorParameters_create()

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=False)

        self.bridge = CvBridge()
        self.pub    = rospy.Publisher('/aruco_pose', PoseStamped, queue_size=1)

        rospy.Subscriber('/usb_cam/image_raw', Image, self.cb_image, queue_size=1,
                         buff_size=2**24)
        rospy.loginfo_throttle(5.0, "[ArUco] 노드 시작 (target ID=%d)", TARGET_ID)

    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn("[ArUco] cv_bridge 오류: %s", e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMS)

        if ids is None:
            return

        for i, mid in enumerate(ids.flatten()):
            if mid != TARGET_ID:
                continue

            c = corners[i][0]  # shape (4,2)

            # ── 픽셀 기반 거리/방향 추정 ──────────────────────────
            # 마커 폭 픽셀
            pixel_w = np.linalg.norm(c[1] - c[0])
            if pixel_w < 1.0:
                continue

            distance = (MARKER_SIZE_M * FOCAL_LEN) / pixel_w  # [m]

            # 마커 중심 픽셀
            cx = c[:, 0].mean()
            cy = c[:, 1].mean()

            # 화면 중심 대비 수평 오프셋 → 방향각 [rad]
            offset_x = cx - IMG_W / 2.0
            bearing  = np.arctan2(offset_x, FOCAL_LEN)  # 좌(-) 우(+)

            # ── PoseStamped 퍼블리시 ──────────────────────────────
            ps = PoseStamped()
            ps.header.stamp    = rospy.Time.now()
            ps.header.frame_id = 'base_link'

            # position.x = 거리[m], position.y = 수평 오프셋[m]
            ps.pose.position.x = distance
            ps.pose.position.y = distance * np.tan(bearing)
            # position.z = 마커 픽셀폭(px). rodong_main 이 ArUco 를 재검출하지 않고
            # 접근 완료(MARKER_CLOSE_PX) 판정에 쓰도록 함께 발행.
            ps.pose.position.z = float(pixel_w)

            # orientation.z = bearing [rad] (yaw 대용)
            ps.pose.orientation.z = bearing
            ps.pose.orientation.w = 1.0

            self.pub.publish(ps)

            rospy.loginfo_throttle(1.0,
                "[ArUco] ID=%d  dist=%.2fm  bearing=%.1fdeg  pixel_w=%.1f",
                mid, distance, np.degrees(bearing), pixel_w)
            break  # ID=1 하나만 처리

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ArucoDetector().run()
