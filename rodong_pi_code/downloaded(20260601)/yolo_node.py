#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_node.py  (RODONG12 - onnxruntime 기반, Pi 4B aarch64)
================================================================
OpenCV 4.5.3 DNN 이 YOLOv8 ONNX 를 지원 못하므로 onnxruntime 사용.

구독: /usb_cam/image_raw (sensor_msgs/Image)
발행: /rodong/yolo (std_msgs/Float32MultiArray)
      data = [class_id, conf, cx_norm, cy_norm, bottom_y_ratio]
        class_id: 0=CLIMB 1=AVOID  (AVOID 만 발행, PUBLISH_ONLY_AVOID=True)
        cx_norm : -1.0(좌) ~ 1.0(우)
        bottom_y_ratio: 박스 하단 y / 프레임 높이
"""

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

# ── 설정 ─────────────────────────────────────────────────────────
MODEL_PATH = os.path.expanduser("~/xycar_ws/src/rodong/models/rodong.onnx")

# rodong_train.py 학습 기준: imgsz=320, 2-class(CLIMB/AVOID)
INPUT_SIZE  = 320
CONF_THRES  = 0.40
NMS_THRES   = 0.45
NUM_CLASSES = 2
CLASS_NAMES = ["CLIMB", "AVOID"]

# Pi CPU 부하 완화: 30fps × SKIP=5 → 약 6회/초 추론
FRAME_SKIP = 5

# AVOID(1) 만 발행. vfh_planner USE_CLIMB=False 와 짝.
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

        # ── onnxruntime 세션 로드 ──
        if not os.path.exists(MODEL_PATH):
            rospy.logwarn("[YOLO] model not found: %s (추론 불가)", MODEL_PATH)
        else:
            try:
                import onnxruntime as ort
                # Pi CPU 전용: inter/intra 스레드 제한으로 발열 완화
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = 2
                opts.inter_op_num_threads = 1
                self.sess = ort.InferenceSession(
                    MODEL_PATH, sess_options=opts,
                    providers=["CPUExecutionProvider"])
                self.input_name = self.sess.get_inputs()[0].name
                rospy.loginfo("[YOLO] model loaded: %s", MODEL_PATH)
            except Exception as e:
                rospy.logerr("[YOLO] 모델 로드 실패: %s", e)

        self.pub = rospy.Publisher('/rodong/yolo', Float32MultiArray, queue_size=1)
        if PUBLISH_DEBUG_IMG:
            self.dbg = rospy.Publisher('/rodong/yolo_debug', Image, queue_size=1)

        rospy.Subscriber('/usb_cam/image_raw', Image, self.cb_image,
                         queue_size=1, buff_size=2**24)
        rospy.loginfo("[YOLO] node started (conf=%.2f, skip=%d)", CONF_THRES, FRAME_SKIP)

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

        # AVOID 만 발행 정책
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

    # ── onnxruntime 추론 ─────────────────────────────────────────
    # 출력: (1, 6, 2100) → squeeze → (6, 2100) → T → (2100, 6)
    def infer(self, frame):
        H, W = frame.shape[:2]

        # 전처리: BGR→RGB, 리사이즈, 정규화, BCHW
        img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        blob = np.transpose(img, (2, 0, 1))[np.newaxis]  # (1,3,320,320)

        # 추론
        pred = self.sess.run(None, {self.input_name: blob})[0]  # (1,6,2100)
        pred = np.squeeze(pred)   # (6, 2100)
        if pred.ndim == 1:
            pred = pred.reshape(-1, 1)
        if pred.shape[0] == 4 + NUM_CLASSES:
            pred = pred.T         # → (2100, 6)

        # 후처리
        boxes, scores, classes = [], [], []
        sx, sy = W / INPUT_SIZE, H / INPUT_SIZE
        for row in pred:
            cls_scores = row[4:4 + NUM_CLASSES]
            cid  = int(np.argmax(cls_scores))
            conf = float(cls_scores[cid])
            if conf < CONF_THRES:
                continue
            cx, cy, bw, bh = row[0], row[1], row[2], row[3]
            boxes.append([int((cx - bw/2) * sx), int((cy - bh/2) * sy),
                          int(bw * sx), int(bh * sy)])
            scores.append(conf)
            classes.append(cid)

        if not boxes:
            return None

        idxs = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRES, NMS_THRES)
        if len(idxs) == 0:
            return None
        idxs = np.array(idxs).flatten()

        # 가장 큰 박스(=가장 가까운 대상) 선택
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
