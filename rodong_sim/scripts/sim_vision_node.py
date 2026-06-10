#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_vision_node.py — vision obstacle detection for Gazebo color boxes (sim-only YOLO shim)
================================================================
The real hardware's yolo_node.py uses rodong.onnx (a real stairs/ramp model), so it
cannot detect Gazebo's solid-color primitive boxes. This node performs saturation-based
detection with the same interface to keep the vision-based avoidance pipeline alive in sim.

Subscribes: /usb_cam/image_raw (sensor_msgs/Image)
Publishes:  /rodong/yolo       (std_msgs/Float32MultiArray)  — same format as yolo_node
      data = [class_id, conf, cx_norm, cy_norm, bottom_y_ratio]
        class_id : fixed 1 (AVOID) (vfh_planner uses it as a virtual obstacle)
        cx_norm  : -1.0(left) ~ 1.0(right)
        bottom_y_ratio : box bottom y / frame height

Principle:
  Gazebo obstacle boxes are highly-saturated solid colors (red/green etc.), while the
  ground/sky are gray (low saturation) and the ArUco marker is black/white (low saturation).
  So an S (saturation) + V (value) threshold in HSV is enough to isolate the colored boxes
  (hue-agnostic → works for any color of obstacle). Among the detected color regions, the
  largest (= nearest) box is published.
"""

import os
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

# Keep a single source of truth with the patch folder's shared constant (rodong_config.CLS_AVOID).
# This runs directly from the rodong_sim package (not via run_app), so add the path directly.
_PATCH = "/workspace/rodong_pi_code/patch(20260602)"
if os.path.isdir(_PATCH):
    sys.path.insert(0, _PATCH)
try:
    import rodong_config as cfg
    AVOID_CLASS = cfg.CLS_AVOID
except Exception:
    AVOID_CLASS = 1


class SimVisionNode:
    def __init__(self):
        rospy.init_node('sim_vision_node', anonymous=False)
        self.bridge  = CvBridge()
        self.frame_i = 0

        # ── tuning parameters (overridable via rosparam) ──
        self.sat_min    = rospy.get_param('~sat_min',    80)    # saturation lower bound
        self.val_min    = rospy.get_param('~val_min',    40)    # value lower bound
        self.min_area   = rospy.get_param('~min_area',   2500)  # minimum box area [px]
        self.frame_skip = rospy.get_param('~frame_skip', 2)     # infer every N frames
        # Forward center window (only a box intruding this width counts as "front blocked").
        # Even with boxes on both sides, if the center is open the car goes straight (= threads the gap).
        self.center_frac = rospy.get_param('~center_frac', 0.20)
        # Proximity gate (bottom ratio). Default 0 = gate off: publish whenever the center is blocked
        # regardless of distance, and "stronger when closer" is handled by vfh_planner's proportional
        # steering (vision_near_lo/hi). (Set >0 to ignore until that distance — for bang-bang mode.)
        self.trigger_bottom = rospy.get_param('~trigger_bottom', 0.0)
        self.publish_debug = rospy.get_param('~publish_debug', False)

        self.pub = rospy.Publisher('/rodong/yolo', Float32MultiArray, queue_size=1)
        if self.publish_debug:
            self.dbg = rospy.Publisher('/rodong/yolo_debug', Image, queue_size=1)

        rospy.Subscriber('/usb_cam/image_raw', Image, self.cb_image,
                         queue_size=1, buff_size=2**24)
        rospy.loginfo("[SimVision] started (sat>=%d val>=%d min_area=%d skip=%d) "
                      "AVOID class=%d", self.sat_min, self.val_min,
                      self.min_area, self.frame_skip, AVOID_CLASS)

    def cb_image(self, msg):
        self.frame_i += 1
        if self.frame_i % self.frame_skip != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5, "[SimVision] cv_bridge: %s", e)
            return

        det = self.detect(frame)
        if det is None:
            return

        cx, cy, w, h, area = det
        H, W = frame.shape[:2]
        cx_norm      = float(np.clip((cx - W / 2.0) / (W / 2.0), -1, 1))
        cy_norm      = float(np.clip((cy - H / 2.0) / (H / 2.0), -1, 1))
        bottom_ratio = float(np.clip((cy + h / 2.0) / H, 0, 1))
        # conf: approximated by screen-area fraction (larger when closer).
        conf = float(np.clip(area / (W * H * 0.12), 0.3, 1.0))

        out = Float32MultiArray()
        out.data = [float(AVOID_CLASS), conf, cx_norm, cy_norm, bottom_ratio]
        self.pub.publish(out)

        rospy.loginfo_throttle(1.0,
            "[SimVision] AVOID conf=%.2f cx=%.2f bottom=%.2f area=%d",
            conf, cx_norm, bottom_ratio, int(area))

        if self.publish_debug:
            x1 = int(cx - w / 2); y1 = int(cy - h / 2)
            x2 = int(cx + w / 2); y2 = int(cy + h / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            self.dbg.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

    # ── saturation-based color box detection (front-center blocking decision) ────
    def detect(self, frame):
        H, W = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        # high saturation and sufficient value → solid-color obstacle. Excludes the gray
        # ground/sky and the black/white ArUco.
        mask = ((s >= self.sat_min) & (v >= self.val_min)).astype(np.uint8) * 255

        # denoise + fill the box
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None

        # Forward center window [cl, cr] — only a box intruding this band counts as "front blocked".
        cl = int((0.5 - self.center_frac / 2.0) * W)
        cr = int((0.5 + self.center_frac / 2.0) * W)

        blocking = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            # if the bbox's horizontal span overlaps the center window, it blocks the front.
            if x < cr and (x + w) > cl:
                blocking.append((area, x, y, w, h))

        # no box blocking the center = the middle is open → go straight (do not avoid).
        if not blocking:
            return None

        # among the boxes blocking the center, pick the largest (= nearest) as the avoid target.
        area, x, y, w, h = max(blocking, key=lambda b: b[0])

        # Proximity gate: if still far (box bottom near the top of the frame), do not avoid yet
        # → steer later and less when closer, reducing center deviation.
        if (y + h) / float(H) < self.trigger_bottom:
            return None

        return (x + w / 2.0, y + h / 2.0, w, h, area)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        SimVisionNode().run()
    except rospy.ROSInterruptException:
        pass
