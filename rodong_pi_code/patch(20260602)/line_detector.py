#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
line_detector.py — floor black-boundary detection (RODONG13)
================================================================
Role: detect the black boundary line on the floor with the USB camera and publish
      left/center/right/near black-pixel ratios to /rodong/boundary so the car does
      not drive outside the boundary. vfh_planner treats this as a virtual obstacle
      and steers back inside the boundary.

Subscribes:
  /usb_cam/image_raw   (sensor_msgs/Image)

Publishes:
  /rodong/boundary     (std_msgs/Float32MultiArray)
      data = [left, center, right, near]
        left/center/right : black-pixel ratio (0~1) of the left/center/right thirds of the ROI (floor area)
        near              : black-pixel ratio (0~1) of the bottom strip of the ROI (closest to the car)
  /rodong/boundary_debug (sensor_msgs/Image, only when ~debug:=true)
      ROI/mask visualization — for tuning thresholds with rqt_image_view.

Tuning parameters (rosparam, set in launch):
  ~black_v_max  (int,  60)    HSV V at/below this → treated as black (raise if bright / lower if shadowy)
  ~black_s_max  (int,  255)   HSV S upper bound (ignored by default). Lower it to exclude dark-colored floors.
  ~roi_top      (float,0.55)  ROI start height ratio (0=top, 1=bottom). Adjust to view only the floor.
  ~near_top     (float,0.85)  near-strip start height ratio (area closest to the car)
  ~min_ratio    (float,0.04)  noise cut on publish (at/below → treated as 0)
  ~max_hz       (float,15.0)  processing rate cap (protects Pi CPU)
  ~debug        (bool, false) publish debug image
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class LineDetector:
    def __init__(self):
        rospy.init_node('line_detector', anonymous=False)
        self.bridge = CvBridge()

        self.black_v_max = int(rospy.get_param('~black_v_max', 60))
        self.black_s_max = int(rospy.get_param('~black_s_max', 255))
        self.roi_top     = float(rospy.get_param('~roi_top', 0.55))
        self.near_top    = float(rospy.get_param('~near_top', 0.85))
        self.min_ratio   = float(rospy.get_param('~min_ratio', 0.04))
        self.max_hz      = float(rospy.get_param('~max_hz', 15.0))
        self.debug       = bool(rospy.get_param('~debug', False))

        self._min_dt   = 1.0 / self.max_hz if self.max_hz > 0 else 0.0
        self._last_t   = rospy.Time(0)
        self._kernel   = np.ones((3, 3), np.uint8)

        self.pub = rospy.Publisher('/rodong/boundary', Float32MultiArray, queue_size=1)
        self.dbg_pub = (rospy.Publisher('/rodong/boundary_debug', Image, queue_size=1)
                        if self.debug else None)

        rospy.Subscriber('/usb_cam/image_raw', Image, self.cb_image,
                         queue_size=1, buff_size=2**24)
        rospy.loginfo('[Line] floor boundary detection started (V<=%d, roi_top=%.2f, debug=%s)',
                      self.black_v_max, self.roi_top, self.debug)

    def cb_image(self, msg):
        now = rospy.Time.now()
        if (now - self._last_t).to_sec() < self._min_dt:
            return                              # rate limit (protect CPU)
        self._last_t = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5, '[Line] cv_bridge error: %s', e)
            return

        h, w = frame.shape[:2]
        y0  = int(h * self.roi_top)
        roi = frame[y0:h, :]
        rh, rw = roi.shape[:2]
        if rh < 2 or rw < 3:
            return

        # ── black mask (HSV: low V = dark) ──────────────────────────────
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0],                       dtype=np.uint8)
        upper = np.array([179, self.black_s_max, self.black_v_max], dtype=np.uint8)
        mask  = cv2.inRange(hsv, lower, upper)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

        # ── left/center/right thirds + near (bottom strip) black-pixel ratio ────
        third = rw // 3

        def ratio(sub):
            return float(np.count_nonzero(sub)) / max(sub.size, 1)

        left   = ratio(mask[:, :third])
        center = ratio(mask[:, third:2 * third])
        right  = ratio(mask[:, 2 * third:])

        denom = max(1.0 - self.roi_top, 1e-3)
        ny0   = int(rh * np.clip((self.near_top - self.roi_top) / denom, 0.0, 0.99))
        near  = ratio(mask[ny0:, :])

        cut = lambda v: v if v >= self.min_ratio else 0.0
        out = Float32MultiArray()
        out.data = [cut(left), cut(center), cut(right), cut(near)]
        self.pub.publish(out)

        rospy.loginfo_throttle(2.0,
            '[Line] L=%.2f C=%.2f R=%.2f near=%.2f', left, center, right, near)

        # ── debug visualization ────────────────────────────────────────
        if self.dbg_pub is not None:
            vis = roi.copy()
            vis[mask > 0] = (0, 0, 255)
            cv2.line(vis, (third, 0), (third, rh), (0, 255, 0), 1)
            cv2.line(vis, (2 * third, 0), (2 * third, rh), (0, 255, 0), 1)
            cv2.line(vis, (0, ny0), (rw, ny0), (255, 0, 0), 1)
            cv2.putText(vis, 'L%.2f C%.2f R%.2f N%.2f' % (left, center, right, near),
                        (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
            try:
                self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))
            except Exception:
                pass

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        LineDetector().run()
    except rospy.ROSInterruptException:
        pass
