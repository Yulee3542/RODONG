#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_node.py  (RODONG12 - onnxruntime based, Pi 4B aarch64)
================================================================
OpenCV 4.5.3 DNN cannot run a YOLOv8 ONNX model, so onnxruntime is used.

Subscribes: /usb_cam/image_raw (sensor_msgs/Image)
Publishes:  /rodong/yolo (std_msgs/Float32MultiArray)
      data = [class_id, conf, cx_norm, cy_norm, bottom_y_ratio]
        class_id: 0=CLIMB 1=AVOID  (only AVOID is published, PUBLISH_ONLY_AVOID=True)
        cx_norm : -1.0(left) ~ 1.0(right)
        bottom_y_ratio: box bottom y / frame height
"""

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

# ── Config ───────────────────────────────────────────────────────
MODEL_PATH = os.path.expanduser("~/xycar_ws/src/rodong/models/rodong.onnx")

# Per rodong_train.py training: imgsz=320, 2-class (CLIMB/AVOID)
INPUT_SIZE  = 320
# CONF_THRES (AVOID detection = confidence at which avoidance starts):
#   The Pi avoidance path (vfh_planner virtual obstacle) has no separate distance/size
#   gate, so avoidance begins the moment YOLO detects the object. So the "box size at
#   which avoidance starts" = "the box size that first crosses this threshold". Lowering
#   it detects smaller (= farther) boxes → avoidance starts earlier (from farther away).
#   Lowered 0.40→0.30 to start avoidance about 1.5× farther. distance↔conf is nonlinear,
#   so fine-tune the exact factor on the real car via rosparam ~conf_thres (lower = earlier).
DEFAULT_CONF_THRES = 0.30
NMS_THRES   = 0.45
NUM_CLASSES = 2
CLASS_NAMES = ["CLIMB", "AVOID"]

# Ease Pi CPU load: 30fps × SKIP=5 → about 6 inferences/sec
FRAME_SKIP = 5

# Publish AVOID(1) only. Pairs with vfh_planner USE_CLIMB=False.
PUBLISH_ONLY_AVOID = True
AVOID_CLASS = 1

PUBLISH_DEBUG_IMG = False


class YoloNode:
    def __init__(self):
        rospy.init_node('yolo_node', anonymous=False)
        self.bridge  = CvBridge()
        self.frame_i = 0
        self.sess    = None
        self.input_name = None
        # Avoidance-start distance knob: lower → detect smaller (farther) box = start avoidance earlier.
        self.conf_thres = float(rospy.get_param('~conf_thres', DEFAULT_CONF_THRES))

        # ── load onnxruntime session ──
        if not os.path.exists(MODEL_PATH):
            rospy.logwarn("[YOLO] model not found: %s (cannot infer)", MODEL_PATH)
        else:
            try:
                import onnxruntime as ort
                # Pi CPU only: limit inter/intra threads to reduce heat
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = 2
                opts.inter_op_num_threads = 1
                self.sess = ort.InferenceSession(
                    MODEL_PATH, sess_options=opts,
                    providers=["CPUExecutionProvider"])
                self.input_name = self.sess.get_inputs()[0].name
                rospy.loginfo("[YOLO] model loaded: %s", MODEL_PATH)
            except Exception as e:
                rospy.logerr("[YOLO] model load failed: %s", e)

        self.pub = rospy.Publisher('/rodong/yolo', Float32MultiArray, queue_size=1)
        if PUBLISH_DEBUG_IMG:
            self.dbg = rospy.Publisher('/rodong/yolo_debug', Image, queue_size=1)

        rospy.Subscriber('/usb_cam/image_raw', Image, self.cb_image,
                         queue_size=1, buff_size=2**24)
        rospy.loginfo("[YOLO] node started (conf=%.2f, skip=%d)", self.conf_thres, FRAME_SKIP)

    def cb_image(self, msg):
        if self.sess is None:
            return
        self.frame_i += 1
        if self.frame_i % FRAME_SKIP != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5, "[YOLO] cv_bridge: %s", e)
            return

        det = self.infer(frame)
        if det is None:
            return

        # AVOID-only publish policy
        if PUBLISH_ONLY_AVOID and det[0] != AVOID_CLASS:
            return

        cls, conf, bx, by, bw, bh = det
        H, W = frame.shape[:2]
        cx_norm      = float(np.clip((bx - W / 2.0) / (W / 2.0), -1, 1))
        cy_norm      = float(np.clip((by - H / 2.0) / (H / 2.0), -1, 1))
        bottom_ratio = float(np.clip((by + bh / 2.0) / H, 0, 1))

        out = Float32MultiArray()
        out.data = [float(cls), float(conf), cx_norm, cy_norm, bottom_ratio]
        self.pub.publish(out)

        rospy.loginfo_throttle(2.0,
            "[YOLO] %s conf=%.2f cx=%.2f bottom=%.2f",
            CLASS_NAMES[cls] if 0 <= cls < len(CLASS_NAMES) else cls,
            conf, cx_norm, bottom_ratio)

        if PUBLISH_DEBUG_IMG:
            x1 = int(bx - bw / 2); y1 = int(by - bh / 2)
            x2 = int(bx + bw / 2); y2 = int(by + bh / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            self.dbg.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

    # ── onnxruntime inference ─────────────────────────────────────
    # output: (1, 6, 2100) → squeeze → (6, 2100) → T → (2100, 6)
    def infer(self, frame):
        H, W = frame.shape[:2]

        # preprocess: BGR→RGB, resize, normalize, BCHW
        img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        blob = np.transpose(img, (2, 0, 1))[np.newaxis]  # (1,3,320,320)

        # inference
        pred = self.sess.run(None, {self.input_name: blob})[0]  # (1,6,2100)
        pred = np.squeeze(pred)   # (6, 2100)
        if pred.ndim == 1:
            pred = pred.reshape(-1, 1)
        if pred.shape[0] == 4 + NUM_CLASSES:
            pred = pred.T         # → (2100, 6)

        # postprocess
        boxes, scores, classes = [], [], []
        sx, sy = W / INPUT_SIZE, H / INPUT_SIZE
        for row in pred:
            cls_scores = row[4:4 + NUM_CLASSES]
            cid  = int(np.argmax(cls_scores))
            conf = float(cls_scores[cid])
            if conf < self.conf_thres:
                continue
            cx, cy, bw, bh = row[0], row[1], row[2], row[3]
            boxes.append([int((cx - bw/2) * sx), int((cy - bh/2) * sy),
                          int(bw * sx), int(bh * sy)])
            scores.append(conf)
            classes.append(cid)

        if not boxes:
            return None

        idxs = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, NMS_THRES)
        if len(idxs) == 0:
            return None
        idxs = np.array(idxs).flatten()

        # pick the largest box (= nearest target)
        best = max(idxs, key=lambda i: boxes[i][2] * boxes[i][3])
        x, y, w, h = boxes[best]
        return (classes[best], scores[best],
                x + w / 2.0, y + h / 2.0, w, h)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        YoloNode().run()
    except rospy.ROSInterruptException:
        pass
