#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_bridge.py — Gazebo ↔ Xycar 인터페이스 브리지 (FSM/회피 로직 검증용)
========================================================================
실제 Xycar 디바이스 드라이버 대신, 시뮬레이션의 일반 토픽을 애플리케이션 코드가
기대하는 토픽으로 변환한다.

  /scan (sensor_msgs/LaserScan)
        → 8개 빔 각도 [-90,-45,45,0,90,135,180,-135]° 샘플링
        → /xycar_ultrasonic (std_msgs/Int32MultiArray, cm)   [rodong 코드가 구독]

  /xycar_motor (xycar_msgs/xycar_motor: speed, angle[deg])   [rodong 코드가 발행]
        → 자전거(bicycle) 모델로 (선속도, yaw속도) 변환
        → /cmd_vel (geometry_msgs/Twist)                      [planar_move 플러그인 구독]

부호 규약: Xycar 는 +angle = 우회전. REP-103(z-up, CCW+)에서 우회전은 음(-)의 yaw.
자전거 모델 yaw_rate = (v/L)*tan(delta), delta=조향. delta = -angle 로 매핑하면
+angle(우) → 음의 yaw → 시계방향 = 실차와 동일. 후진(v<0) 시 부호도 자연히 뒤집힌다
(→ 후진 중 조향/직진복귀 동작도 실차와 같은 방향으로 검증된다).
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

# rodong_config.BEAM_ANGLES 와 동일 (idx0=좌 … idx7=좌후)
BEAM_ANGLES_DEG = [-90, -45, 45, 0, 90, 135, 180, -135]


class SimBridge(object):
    def __init__(self):
        rospy.init_node('sim_bridge')

        # 튜닝 파라미터 (launch 에서 override 가능)
        self.speed_to_ms   = rospy.get_param('~speed_to_ms', 0.03)    # 모터단위 → m/s (25→0.75)
        self.wheelbase     = rospy.get_param('~wheelbase', 0.30)      # 자전거 모델 L [m]
        self.max_steer_deg = rospy.get_param('~max_steer_deg', 60.0)  # tan 발산 방지 클램프
        self.max_cm        = rospy.get_param('~max_cm', 300)          # 무반사 시 보고 거리 [cm]

        self.scan = None

        self.son_pub = rospy.Publisher('/xycar_ultrasonic', Int32MultiArray, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('/scan', LaserScan, self.cb_scan, queue_size=1)
        rospy.Subscriber('/xycar_motor', xycar_motor, self.cb_motor, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.cb_timer)   # 20Hz 로 초음파 발행
        rospy.loginfo('[sim_bridge] ready — /scan→/xycar_ultrasonic, /xycar_motor→/cmd_vel')

    # ── 라이다 → 8빔 초음파 ──────────────────────────────────────────
    def cb_scan(self, msg):
        self.scan = msg

    def _range_cm_at(self, deg):
        s = self.scan
        ang = math.radians(deg)
        # [angle_min, angle_max] 범위로 정규화
        while ang < s.angle_min:
            ang += 2 * math.pi
        while ang > s.angle_max:
            ang -= 2 * math.pi
        idx = int(round((ang - s.angle_min) / s.angle_increment))
        if idx < 0 or idx >= len(s.ranges):
            return self.max_cm
        r = s.ranges[idx]
        if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0:
            return self.max_cm
        return min(int(r * 100.0), self.max_cm)

    def cb_timer(self, _evt):
        if self.scan is None:
            return
        arr = Int32MultiArray()
        arr.data = [self._range_cm_at(d) for d in BEAM_ANGLES_DEG]
        self.son_pub.publish(arr)

    # ── 모터 명령 → cmd_vel (자전거 모델) ────────────────────────────
    def cb_motor(self, msg):
        v = msg.speed * self.speed_to_ms
        steer = max(-self.max_steer_deg, min(self.max_steer_deg, msg.angle))
        yaw_rate = -(v / self.wheelbase) * math.tan(math.radians(steer))

        tw = Twist()
        tw.linear.x  = v
        tw.angular.z = yaw_rate
        self.cmd_pub.publish(tw)


if __name__ == '__main__':
    try:
        SimBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
