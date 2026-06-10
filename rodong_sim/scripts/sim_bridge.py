#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_bridge.py — Gazebo ↔ Xycar interface bridge (for FSM/avoidance verification)
========================================================================
Instead of the real Xycar device drivers, this converts the simulation's generic
topics into the topics the application code expects.

  /scan (sensor_msgs/LaserScan)
        → sample 8 beam angles [-90,-45,45,0,90,135,180,-135]°
        → /xycar_ultrasonic (std_msgs/Int32MultiArray, cm)   [subscribed by rodong code]

  /xycar_motor (xycar_msgs/xycar_motor: speed, angle[deg])   [published by rodong code]
        → convert to (linear velocity, yaw rate) via a bicycle model
        → /cmd_vel (geometry_msgs/Twist)                      [subscribed by the planar_move plugin]

Sign convention: Xycar uses +angle = right turn. In REP-103 (z-up, CCW+) a right turn is
negative yaw. Bicycle model yaw_rate = (v/L)*tan(delta), delta=steering. Mapping delta = -angle
makes +angle (right) → negative yaw → clockwise = same as the real car. In reverse (v<0) the
sign flips naturally (→ steering/straightening while reversing is verified in the same direction
as the real car).
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

# Same as rodong_config.BEAM_ANGLES (idx0=left … idx7=rear-left)
BEAM_ANGLES_DEG = [-90, -45, 45, 0, 90, 135, 180, -135]


class SimBridge(object):
    def __init__(self):
        rospy.init_node('sim_bridge')

        # Tuning parameters (overridable in launch)
        self.speed_to_ms   = rospy.get_param('~speed_to_ms', 0.03)    # motor units → m/s (25→0.75)
        self.wheelbase     = rospy.get_param('~wheelbase', 0.30)      # bicycle model L [m]
        self.max_steer_deg = rospy.get_param('~max_steer_deg', 60.0)  # clamp to avoid tan blow-up
        self.max_cm        = rospy.get_param('~max_cm', 300)          # reported distance on no return [cm]

        self.scan = None

        self.son_pub = rospy.Publisher('/xycar_ultrasonic', Int32MultiArray, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('/scan', LaserScan, self.cb_scan, queue_size=1)
        rospy.Subscriber('/xycar_motor', xycar_motor, self.cb_motor, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.cb_timer)   # publish ultrasonic at 20Hz
        rospy.loginfo('[sim_bridge] ready — /scan→/xycar_ultrasonic, /xycar_motor→/cmd_vel')

    # ── lidar → 8-beam ultrasonic ────────────────────────────────────
    def cb_scan(self, msg):
        self.scan = msg

    def _range_cm_at(self, deg):
        s = self.scan
        ang = math.radians(deg)
        # normalize into the [angle_min, angle_max] range
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

    # ── motor command → cmd_vel (bicycle model) ──────────────────────
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
