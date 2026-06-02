#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
line_detector.py — 바닥 검은 경계선 검출 (RODONG13)
================================================================
역할: USB 카메라로 바닥의 검은 경계선을 검출하여, 차가 경계 밖으로
      나가지 않도록 /rodong/boundary 로 좌/중/우/근접 검은픽셀 비율을 발행.
      vfh_planner 가 이를 가상 장애물로 받아 경계 안쪽으로 조향한다.

구독:
  /usb_cam/image_raw   (sensor_msgs/Image)

발행:
  /rodong/boundary     (std_msgs/Float32MultiArray)
      data = [left, center, right, near]
        left/center/right : ROI(바닥 영역) 좌/중/우 1/3 구역 검은픽셀 비율(0~1)
        near              : ROI 최하단(차에 가장 가까운) 띠의 검은픽셀 비율(0~1)
  /rodong/boundary_debug (sensor_msgs/Image, ~debug:=true 일 때만)
      ROI/마스크 시각화 — rqt_image_view 로 임계값 튜닝용.

튜닝 파라미터 (rosparam, launch 에서 지정):
  ~black_v_max  (int,  60)    HSV V 가 이 값 이하 → 검정으로 간주 (밝으면↑/그림자 많으면↓)
  ~black_s_max  (int,  255)   HSV S 상한 (기본 무시). 짙은 색 바닥 배제 시 낮춤.
  ~roi_top      (float,0.55)  ROI 시작 높이비 (0=상단,1=하단). 바닥만 보도록 조정.
  ~near_top     (float,0.85)  near 띠 시작 높이비 (차에 가장 가까운 영역)
  ~min_ratio    (float,0.04)  발행 시 노이즈 컷 (이하 0 처리)
  ~max_hz       (float,15.0)  처리 주기 상한 (Pi CPU 보호)
  ~debug        (bool, false) 디버그 이미지 발행
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
        rospy.loginfo('[Line] 바닥 경계선 검출 시작 (V<=%d, roi_top=%.2f, debug=%s)',
                      self.black_v_max, self.roi_top, self.debug)

    def cb_image(self, msg):
        now = rospy.Time.now()
        if (now - self._last_t).to_sec() < self._min_dt:
            return                              # 처리 주기 제한 (CPU 보호)
        self._last_t = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5, '[Line] cv_bridge 오류: %s', e)
            return

        h, w = frame.shape[:2]
        y0  = int(h * self.roi_top)
        roi = frame[y0:h, :]
        rh, rw = roi.shape[:2]
        if rh < 2 or rw < 3:
            return

        # ── 검정 마스크 (HSV: V 낮음 = 어두움) ──────────────────────────
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0],                       dtype=np.uint8)
        upper = np.array([179, self.black_s_max, self.black_v_max], dtype=np.uint8)
        mask  = cv2.inRange(hsv, lower, upper)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

        # ── 좌/중/우 1/3 + near(하단 띠) 검은픽셀 비율 ──────────────────
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

        # ── 디버그 시각화 ──────────────────────────────────────────────
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
