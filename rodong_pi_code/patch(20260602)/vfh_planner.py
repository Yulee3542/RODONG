#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
vfh_planner.py  (RODONG13 - sensor-fusion active avoidance)
================================================================
Role: the RODONG avoidance brain. Fuses ultrasonic + ArUco goal + YOLO + floor
      boundary into a VFH+ algorithm to compute (speed, angle) → /rodong/vfh_cmd.

Subscribes:
  /xycar_ultrasonic  (std_msgs/Int32MultiArray, 8 beams)
  /aruco_pose        (geometry_msgs/PoseStamped)      - VFH+ goal
  /rodong/yolo       (std_msgs/Float32MultiArray)     - camera decision (optional)
                      data = [class_id, conf, cx_norm, cy_norm, bottom_y_ratio]
  /rodong/boundary   (std_msgs/Float32MultiArray)     - floor boundary line (optional)
                      data = [left, center, right, near]

Publishes:
  /rodong/vfh_cmd    (xycar_msgs/xycar_motor)

Design notes:
  - Constants / sonar-beam / histogram logic split into rodong_config / rodong_sonar
    (single source of truth).
  - rodong_main.py subscribes to this topic and only forwards to /xycar_motor in the
    BUG_DRIVE state.
  - If the goal (ArUco) is visible, prefer its direction; otherwise aim straight
    (sector 3) and avoid.
  - YOLO AVOID / floor boundary add a virtual obstacle to the matching sectors
    (active avoidance).
"""

import os
import sys
import math
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from xycar_msgs.msg import xycar_motor

# catkin runs nodes from the devel/lib wrapper, so scripts/ is not on sys.path.
# Add this folder so the shared modules (rodong_config etc.) can be imported.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rodong_config as cfg
from rodong_geometry import angle_to_sector, clip
from rodong_sonar import build_histogram, select_sector, front_min


class VFHPlanner:
    def __init__(self):
        rospy.init_node('vfh_planner', anonymous=False)

        self.sonar = [999] * 8

        # ArUco goal
        self.goal_bearing  = 0.0    # [rad], left(-)/right(+)
        self.goal_distance = None   # [m]
        self.last_goal_t   = None

        # YOLO decision
        self.yolo_cls    = -1
        self.yolo_cx     = 0.0      # -1.0 ~ 1.0
        self.yolo_bottom = 0.0
        self.last_yolo_t = None

        # Floor boundary line [left, center, right, near] black-pixel ratio
        self.bound        = [0.0, 0.0, 0.0, 0.0]
        self.last_bound_t = None

        self.prev_steer = 0.0       # last chosen steering angle [deg] (for smoothing)

        # ── Vision proportional steering (gentle avoidance) ──
        # Default off → hardware behavior unchanged. When enabled (in sim), as a YOLO
        # AVOID box gets closer, steer by a gentle angle below AVOID_TRIG_ANG so that
        # rodong_main follows smoothly without latching → minimal center deviation.
        # Far → 0 (straight); the closer it is, the larger the proportional angle.
        self.vis_enable   = rospy.get_param('~vision_steer_enable', False)
        self.vis_max      = rospy.get_param('~vision_steer_max', 30.0)   # [deg] < AVOID_TRIG_ANG(32) → no latch
        self.vis_near_lo  = rospy.get_param('~vision_near_lo', 0.40)     # start steering once bottom >= this (early)
        self.vis_near_hi  = rospy.get_param('~vision_near_hi', 0.70)     # max angle around here

        self.pub = rospy.Publisher('/rodong/vfh_cmd', xycar_motor, queue_size=1)

        rospy.Subscriber('/xycar_ultrasonic', Int32MultiArray,
                         self.cb_sonar, queue_size=1)
        rospy.Subscriber('/aruco_pose', PoseStamped,
                         self.cb_goal, queue_size=1)
        rospy.Subscriber('/rodong/yolo', Float32MultiArray,
                         self.cb_yolo, queue_size=1)
        rospy.Subscriber('/rodong/boundary', Float32MultiArray,
                         self.cb_boundary, queue_size=1)

        rospy.loginfo("[VFH+] sensor-fusion planner started (threshold=%.0fcm)",
                      cfg.THRESHOLD)
        rospy.Timer(rospy.Duration(0.1), self.cb_timer)   # 10Hz

    # ── Callbacks ────────────────────────────────────────────────
    def cb_sonar(self, msg):
        self.sonar = list(msg.data[:8])

    def cb_goal(self, msg):
        self.goal_bearing  = msg.pose.orientation.z   # [rad]
        self.goal_distance = msg.pose.position.x      # [m]
        self.last_goal_t   = rospy.Time.now()

    def cb_yolo(self, msg):
        d = list(msg.data)
        if len(d) >= 5:
            self.yolo_cls    = int(d[0])
            self.yolo_cx     = d[2]
            self.yolo_bottom = d[4]
            self.last_yolo_t = rospy.Time.now()

    def cb_boundary(self, msg):
        d = list(msg.data)
        if len(d) >= 4:
            self.bound        = d[:4]
            self.last_bound_t = rospy.Time.now()

    # ── Validity check ───────────────────────────────────────────
    def _valid(self, t, timeout):
        return t is not None and (rospy.Time.now() - t).to_sec() < timeout

    # ── Main loop (10Hz) ─────────────────────────────────────────
    def cb_timer(self, event):
        goal_valid = self._valid(self.last_goal_t, cfg.GOAL_TIMEOUT)
        yolo_valid = self._valid(self.last_yolo_t, cfg.YOLO_TIMEOUT)

        # ── Interpret YOLO intent ────────────────────────────────
        # climb_now: treat the front object as something to climb over → relax obstacle judgment
        # avoid_dir: on-screen direction to avoid (left/right) → add a virtual obstacle
        climb_now = False
        avoid_dir = None   # 'left' / 'right' / 'center'
        if yolo_valid:
            if (cfg.USE_CLIMB and self.yolo_cls == cfg.CLS_CLIMB
                    and self.yolo_bottom >= cfg.CLIMB_BOTTOM_RATIO):
                climb_now = True
            elif self.yolo_cls == cfg.CLS_AVOID:
                if self.yolo_cx < -0.15:
                    avoid_dir = 'left'
                elif self.yolo_cx > 0.15:
                    avoid_dir = 'right'
                else:
                    avoid_dir = 'center'

        # ── 1. Polar histogram (8 sonar → 7 sectors) ─────────────
        hist = build_histogram(self.sonar, climb_now=climb_now)

        # ── YOLO AVOID → add a virtual obstacle ──────────────────
        if avoid_dir == 'left':
            hist[0] += 1.0; hist[1] += 1.0; hist[2] += 0.7
        elif avoid_dir == 'right':
            hist[6] += 1.0; hist[5] += 1.0; hist[4] += 0.7
        elif avoid_dir == 'center':
            hist[2] += 0.8; hist[3] += 1.0; hist[4] += 0.8

        # ── Floor black boundary → virtual obstacle (stay inside the boundary) ────
        # Block the sector where the boundary is seen → steer the other (inner) way.
        if self._valid(self.last_bound_t, cfg.BOUND_TIMEOUT):
            bl, bc, br, bn = self.bound
            if bl > cfg.BOUND_TH:              # left boundary → block left → steer right
                hist[0] += 1.5; hist[1] += 1.2; hist[2] += 0.6
            if br > cfg.BOUND_TH:              # right boundary → block right → steer left
                hist[6] += 1.5; hist[5] += 1.2; hist[4] += 0.6
            if bc > cfg.BOUND_TH:              # center boundary
                hist[2] += 0.8; hist[3] += 1.2; hist[4] += 0.8
            if bn > cfg.BOUND_NEAR_TH:         # boundary imminent right in front → strongly block center
                hist[2] += 1.5; hist[3] += 2.0; hist[4] += 1.5
            rospy.loginfo_throttle(2.0,
                "[VFH+] boundary L=%.2f C=%.2f R=%.2f near=%.2f",
                bl, bc, br, bn)

        # ── 2. Determine the goal sector ─────────────────────────
        goal_deg = math.degrees(self.goal_bearing) if goal_valid else 0.0
        goal_sector = angle_to_sector(goal_deg)
        if goal_sector is None:
            goal_sector = 3

        # ── 3. Evaluate candidate sector costs ───────────────────
        best_sector = select_sector(hist, goal_sector, self.prev_steer)

        # ── 4. Motor command ─────────────────────────────────────
        if best_sector is None:
            # everything blocked → reverse
            rospy.logwarn_throttle(1.0, "[VFH+] all blocked → reverse")
            self.prev_steer = 0.0
            self.publish(cfg.SPEED_BACK, 0)
            return

        steer = int(clip(cfg.SECTOR_ANGLE[best_sector], -cfg.ANGLE_MAX, cfg.ANGLE_MAX))

        # ── Vision proportional steering: gently avoid as a front AVOID gets closer ──
        # Instead of the histogram (±90 bang-bang), emit a gentle angle below
        # AVOID_TRIG_ANG so rodong_main follows smoothly without latching the avoid sub-FSM.
        vis_steer = self._vision_steer(yolo_valid)
        if vis_steer is not None:
            steer = int(vis_steer)

        self.prev_steer = steer

        # Speed: slow down if the front is close
        min_front = front_min(self.sonar)
        speed = cfg.SPEED_DRIVE
        if climb_now:
            speed = cfg.SPEED_DRIVE    # slow down when approaching a climb target (currently same value)

        self.publish(speed, steer)

        rospy.loginfo_throttle(5.0,
            "[VFH+] goal=%.0fdeg sec=%d steer=%d spd=%d front=%.0fcm "
            "climb=%s avoid=%s",
            goal_deg, best_sector, steer, speed, min_front,
            climb_now, avoid_dir)

    def _vision_steer(self, yolo_valid):
        """Gentle proportional steering[deg] for a front AVOID box, or None (not applied).
        - Far (bottom < near_lo) → None → keep going straight.
        - Closer (near_lo→near_hi) → proportional 0→vis_max, sign opposite the obstacle.
        - Head-on (cx≈0) → steer toward the more open sonar side."""
        if not self.vis_enable or not yolo_valid or self.yolo_cls != cfg.CLS_AVOID:
            return None
        close = (self.yolo_bottom - self.vis_near_lo) / max(1e-3, self.vis_near_hi - self.vis_near_lo)
        close = clip(close, 0.0, 1.0)
        if close <= 0.0:
            return None
        if abs(self.yolo_cx) < 0.12:
            # head-on obstacle → toward the more open side (left vs right beam). Tie → left.
            left_room  = min(self.sonar[cfg.SONAR_LEFT],  self.sonar[cfg.SONAR_FRONT_L])
            right_room = min(self.sonar[cfg.SONAR_RIGHT], self.sonar[cfg.SONAR_FRONT_R])
            direction = -1.0 if left_room >= right_room else +1.0
        else:
            direction = -1.0 if self.yolo_cx > 0 else +1.0   # opposite the obstacle
        return clip(direction * self.vis_max * close, -self.vis_max, self.vis_max)

    def publish(self, speed, angle):
        msg = xycar_motor()
        msg.speed = int(speed)
        msg.angle = int(angle)
        self.pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        VFHPlanner().run()
    except rospy.ROSInterruptException:
        pass
