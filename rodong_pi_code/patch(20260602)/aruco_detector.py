#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
aruco_detector.py
- subscribes /usb_cam/image_raw
- detects ArUco ID=1 (old API, OpenCV 4.5)
- estimates pixel-based distance/bearing → publishes /aruco_pose (PoseStamped)
- does not publish /aruco_pose when nothing is detected
"""

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

# ── Camera parameters (estimates before calibration) ──────────────
IMG_W = 640
IMG_H = 480
FOCAL_LEN = 600.0          # estimated focal length in pixels
MARKER_SIZE_M = 0.15       # real marker size [m]

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
        rospy.loginfo_throttle(5.0, "[ArUco] node started (target ID=%d)", TARGET_ID)

    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn("[ArUco] cv_bridge error: %s", e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMS)

        if ids is None:
            return

        for i, mid in enumerate(ids.flatten()):
            if mid != TARGET_ID:
                continue

            c = corners[i][0]  # shape (4,2)

            # ── pixel-based distance/bearing estimate ──────────────
            # marker width in pixels
            pixel_w = np.linalg.norm(c[1] - c[0])
            if pixel_w < 1.0:
                continue

            distance = (MARKER_SIZE_M * FOCAL_LEN) / pixel_w  # [m]

            # marker center pixel
            cx = c[:, 0].mean()
            cy = c[:, 1].mean()

            # horizontal offset from screen center → bearing [rad]
            offset_x = cx - IMG_W / 2.0
            bearing  = np.arctan2(offset_x, FOCAL_LEN)  # left(-) right(+)

            # ── publish PoseStamped ────────────────────────────────
            ps = PoseStamped()
            ps.header.stamp    = rospy.Time.now()
            ps.header.frame_id = 'base_link'

            # position.x = distance[m], position.y = horizontal offset[m]
            ps.pose.position.x = distance
            ps.pose.position.y = distance * np.tan(bearing)
            # position.z = marker pixel width(px). Published so rodong_main can decide
            # approach completion (MARKER_CLOSE_PX) without re-detecting the ArUco.
            ps.pose.position.z = float(pixel_w)

            # orientation.z = bearing [rad] (used as yaw)
            ps.pose.orientation.z = bearing
            ps.pose.orientation.w = 1.0

            self.pub.publish(ps)

            rospy.loginfo_throttle(1.0,
                "[ArUco] ID=%d  dist=%.2fm  bearing=%.1fdeg  pixel_w=%.1f",
                mid, distance, np.degrees(bearing), pixel_w)
            break  # process only ID=1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ArucoDetector().run()
