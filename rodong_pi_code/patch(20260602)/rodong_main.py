#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rodong_main.py  —  RODONG13 rev4
================================================================
Changes (rev4):
  + Split shared config/geometry/sonar logic into rodong_config / rodong_geometry /
    rodong_sonar (single source of constants shared with vfh_planner, unit-testable).
  + Removed duplicate ArUco detection: no longer decodes/detects /usb_cam/image_raw
    directly; subscribes to /aruco_pose published by aruco_detector (saves Pi CPU).
  + Closed-loop control via PID: marker approach (bearing→steering), heading
    recovery (yaw error→steering).

Previous (rev3):
  1. Longer avoidance-steering hold / 2. IMU yaw heading recovery (RECOVER)
  3. U-turn K-turn / 4. Steer-toward-obstacle reverse / STOP abort

State machine: IDLE → BUG_DRIVE ⇄ {MARKER_APPROACH → UTURN} / MANUAL_DRIVE / STOP

────────────────────────────────────────────────────────────────
[SIM/REAL distinction — archiving note]
The core driving logic in this file targets the real Xycar hardware and is shared
with the simulator. Sim-only behavior is enabled via rosparams only, and the
default always preserves the real-hardware behavior:
  • ~return_yaw_lock (default False)
      - real: disabled → keep the existing free return drive (vfh-following) after a U-turn.
      - sim (true in rodong_sim/launch/_demo_two_box.launch):
        hold the return drive at a fixed global yaw (start heading + 180°) to prevent
        heading drift.
All sim-only sections in the code are tagged with '[SIM-ONLY]'.
Speed is unified via rodong_config.SPEED (single variable) — applied to both real and sim.
"""

import os
import sys
import math
import time
import threading
import rospy
import tf
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, PoseStamped
from xycar_msgs.msg import xycar_motor

# catkin runs nodes from the devel/lib wrapper, so scripts/ is not on sys.path.
# Add this folder so the shared modules (rodong_config etc.) can be imported.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rodong_config as cfg
from rodong_geometry import clip, yaw_diff, yaw_signed_diff
from rodong_sonar import front_min, rear_min
from rodong_control import PID


# ── States ──────────────────────────────────────────────────────────────────
class State:
    IDLE            = 'IDLE'
    BUG_DRIVE       = 'BUG_DRIVE'
    MARKER_APPROACH = 'MARKER_APPROACH'
    UTURN           = 'UTURN'
    MANUAL_DRIVE    = 'MANUAL_DRIVE'
    STOP            = 'STOP'


# ── Avoidance sub-phases ─────────────────────────────────────────────────────
class Avoid:
    NONE    = 'NONE'      # normal driving (VFH following)
    STEER   = 'STEER'     # hold avoidance steering
    RECOVER = 'RECOVER'   # return to the original heading


class RodongMain:
    def __init__(self):
        rospy.init_node('rodong_main', anonymous=False)

        self.state       = State.IDLE
        self.prev_state  = None
        self.sonar       = [999] * 8

        # Marker (subscribes /aruco_pose — does not detect directly)
        self.marker_bearing  = 0.0     # [rad] left(-)/right(+)
        self.marker_pixel_w  = 0.0     # [px]
        self.last_marker_t   = None
        self.marker_seen     = 0

        self.vfh_speed   = 0
        self.vfh_angle   = 0

        # Avoidance sub-FSM
        self.avoid_phase   = Avoid.NONE
        self.avoid_ang     = 0          # latched avoidance steering angle
        self.avoid_t0      = None       # avoidance-steering start time
        self.recover_t0    = None       # recovery start time
        self.heading_target = None      # original yaw to recover to [deg]
        # U-turn reference heading: the yaw *before* entering marker approach
        # (= original driving direction). The U-turn must rotate 170° relative to
        # this value (not the heading skewed by the approach) to end up exactly
        # opposite the original direction of travel.
        self.uturn_ref_yaw = None

        self.manual_goal = None
        self.manual_done = False
        self.reversing   = False

        # IMU
        self.yaw_deg   = 0.0
        self.imu_ready = False

        # ── PID controllers ──
        self.marker_pid  = PID(**cfg.MARKER_PID)    # bearing[rad] → steering[deg]
        self.recover_pid = PID(**cfg.RECOVER_PID)   # yaw error[deg] → steering[deg]
        self._marker_prev_t  = None
        self._recover_prev_t = None

        # ── [SIM-ONLY] hold return yaw after a U-turn (prevents heading drift) ──────
        #   Real hardware: return_yaw_lock defaults to False → all related branches
        #                  inactive → existing behavior (free return after U-turn) kept.
        #   Sim: true in _demo_two_box.launch → hold the return at global yaw (start+180°).
        self.return_yaw_lock   = rospy.get_param('~return_yaw_lock', False)
        self.start_yaw         = None               # global yaw at the first BUG_DRIVE entry
        self.return_yaw_target = None               # return yaw to hold after U-turn = start_yaw+180
        self.return_pid        = PID(**cfg.RECOVER_PID)
        self._return_prev_t    = None

        # Publisher
        self.motor_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        # Subscribers
        rospy.Subscriber('/xycar_ultrasonic',   Int32MultiArray, self.cb_sonar)
        rospy.Subscriber('/aruco_pose',         PoseStamped,     self.cb_aruco)
        rospy.Subscriber('/rodong/vfh_cmd',     xycar_motor,     self.cb_vfh)
        rospy.Subscriber('/rodong/cmd',         String,          self.cb_cmd)
        rospy.Subscriber('/rodong/manual_goal', Point,           self.cb_manual_goal)
        rospy.Subscriber('/imu/data',           Imu,             self.cb_imu)

        rospy.loginfo('[Main] RODONG13 rev4 — IDLE. Press "i" to start.')

    # ── Callbacks ──────────────────────────────────────────────────────────
    def cb_sonar(self, msg):
        self.sonar = list(msg.data)

    def cb_aruco(self, msg):
        # aruco_detector publishes only when a marker is detected → received = visible.
        self.marker_bearing = msg.pose.orientation.z   # [rad]
        self.marker_pixel_w = msg.pose.position.z      # [px]
        self.last_marker_t  = rospy.Time.now()

    def cb_vfh(self, msg):
        self.vfh_speed = msg.speed
        self.vfh_angle = msg.angle

    def cb_imu(self, msg):
        q = msg.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_deg   = math.degrees(euler[2])
        self.imu_ready = True

    def cb_cmd(self, msg):
        cmd = msg.data.strip().upper()
        rospy.loginfo('[Main] CMD: %s', cmd)
        if cmd == 'INIT':
            if self.state in (State.IDLE, State.STOP):
                self.set_state(State.BUG_DRIVE)
            elif self.state == State.MANUAL_DRIVE:
                self.manual_goal = None
                self.set_state(State.BUG_DRIVE)
        elif cmd == 'STOP':
            self.set_state(State.STOP)     # running uturn/reverse/avoid loops watch this state and abort
        elif cmd == 'UTURN':
            self.set_state(State.UTURN)
        elif cmd == 'MANUAL':
            self.set_state(State.MANUAL_DRIVE)
            self.manual_goal = None
            self.manual_done = True

    def cb_manual_goal(self, msg):
        if self.state == State.MANUAL_DRIVE:
            rospy.loginfo('[Main] new goal: x=%.1f y=%.1f cm', msg.x, msg.y)
            self.manual_goal = msg
            self.manual_done = False

    # ── Utilities ──────────────────────────────────────────────────────────
    def set_state(self, s):
        if s != self.state:
            rospy.loginfo('[Main] %s → %s', self.state, s)
            self.prev_state = self.state
            # When toggling with BUG_DRIVE due to an obstacle during approach, the
            # approach is logically continuous, so do not reset the marker PID
            # (frequent resets → derivative kick → steering jitter).
            marker_toggle = {self.state, s} == {State.BUG_DRIVE, State.MARKER_APPROACH}
            self.state = s
            # Always reset the avoidance sub-FSM on state change (start a clean avoid).
            self._reset_avoid()
            if not marker_toggle:
                self.marker_pid.reset()
                self._marker_prev_t = None

    def _reset_avoid(self):
        self.avoid_phase    = Avoid.NONE
        self.avoid_ang      = 0
        self.avoid_t0       = None
        self.recover_t0     = None
        self.heading_target = None
        self.recover_pid.reset()
        self._recover_prev_t = None

    def _valid(self, t, timeout):
        return t is not None and (rospy.Time.now() - t).to_sec() < timeout

    def marker_visible(self):
        return self._valid(self.last_marker_t, cfg.MARKER_TIMEOUT)

    def _dt(self, attr):
        """Elapsed[s] since the previous time stored in attr. First call returns the loop period (0.05)."""
        now  = rospy.Time.now()
        prev = getattr(self, attr)
        setattr(self, attr, now)
        return (now - prev).to_sec() if prev is not None else 0.05

    def drive(self, speed, angle):
        msg = xycar_motor()
        msg.speed = int(speed)
        msg.angle = int(clip(angle, -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
        self.motor_pub.publish(msg)

    def stop_motor(self):
        self.drive(0, 0)

    def _esc_reverse_init(self):
        """Handle the reverse ESC deadband: neutral for 1s before reverse commands work."""
        self.drive(0, 0)
        rospy.sleep(1.0)

    def is_front_blocked(self):
        # Emergency reverse is judged by the front (center) beam ONLY. If the ±45°
        # side beams were included, the side beam would catch an obstacle the car is
        # passing alongside → panic-reverse → recover to original heading (toward the
        # obstacle) → re-collision loop. Side obstacles are avoided by steering
        # (VFH/vision); emergency stop is only for an imminent head-on collision.
        d = self.sonar[cfg.SONAR_FRONT]
        return 0 < d < cfg.SONAR_EMERGENCY

    # ══════════════════════════════════════════════════════════════════════
    # Reverse — back up while steering toward the nearest front obstacle
    # ══════════════════════════════════════════════════════════════════════
    def _reverse_steer_from_front(self):
        """Pick the steering angle toward the nearest front obstacle. While reversing,
        the car's front swings the other way, moving away from the obstacle."""
        lf = self.sonar[cfg.SONAR_FRONT_L] if self.sonar[cfg.SONAR_FRONT_L] > 0 else 999
        rf = self.sonar[cfg.SONAR_FRONT_R] if self.sonar[cfg.SONAR_FRONT_R] > 0 else 999
        cf = self.sonar[cfg.SONAR_FRONT]   if self.sonar[cfg.SONAR_FRONT]   > 0 else 999

        if rf <= lf and rf <= cf:
            rospy.loginfo('[Reverse] front-right(%.0fcm) → reverse with right steer(+%d°)', rf, cfg.AVOID_FULL_ANG)
            return cfg.AVOID_FULL_ANG
        if lf < rf and lf <= cf:
            rospy.loginfo('[Reverse] front-left(%.0fcm) → reverse with left steer(-%d°)', lf, cfg.AVOID_FULL_ANG)
            return -cfg.AVOID_FULL_ANG
        # center obstacle → toward the nearer front side
        steer = cfg.AVOID_FULL_ANG if rf <= lf else -cfg.AVOID_FULL_ANG
        rospy.loginfo('[Reverse] front(%.0fcm) → steer %d°', cf, steer)
        return steer

    def reverse_motor(self, speed=None, target_front_cm=30):
        """Reverse until the front clears target_front_cm. (Runs in a separate thread.)
        Backs up while steering toward the obstacle, and avoids rear obstacles.
        Schedules IMU heading recovery on completion."""
        if speed is None:
            speed = cfg.SPEED_DRIVE
        self.reversing = True
        steer = self._reverse_steer_from_front()
        rospy.loginfo('[Reverse] start — until front %.0fcm, steer=%d°',
                      target_front_cm, steer)

        # ESC reverse-enable sequence (deadband handling)
        self.drive(0, 0);             time.sleep(0.5)
        self.drive(-abs(speed), 0);   time.sleep(0.5)
        self.drive(0, 0);             time.sleep(0.5)

        rate    = rospy.Rate(20)
        timeout = time.time() + 8.0

        while (not rospy.is_shutdown()
               and self.state not in (State.STOP, State.IDLE)
               and time.time() < timeout):

            front = front_min(self.sonar)
            if front >= target_front_cm:
                rospy.loginfo('[Reverse] front cleared %.0fcm → done', front)
                break

            rear_c = self.sonar[cfg.SONAR_REAR]     # rear (180°)
            rear_r = self.sonar[cfg.SONAR_REAR_R]   # rear-right (+135°)
            rear_l = self.sonar[cfg.SONAR_REAR_L]   # rear-left (-135°)

            if 0 < rear_c < cfg.SONAR_REVERSE:
                rospy.logwarn('[Reverse] rear center blocked → stop')
                break

            # Keep the reverse steering for front-obstacle avoidance, but relax to
            # straight reverse if the rear on the steering side becomes blocked.
            cur = steer
            if steer > 0 and 0 < rear_r < cfg.SONAR_REVERSE:
                cur = 0
            elif steer < 0 and 0 < rear_l < cfg.SONAR_REVERSE:
                cur = 0

            self.drive(-abs(speed), cur)
            rate.sleep()

        self.stop_motor()

        # Straighten the steering held during reverse to center (0°) and give the
        # servo time to physically settle at center — so that the wheels are not
        # cranked when driving forward again.
        self.drive(0, 0)
        time.sleep(0.4)

        self.reversing = False

        # After reverse: do NOT go straight to RECOVER (return to original heading).
        # The original heading is toward the obstacle that just forced the reverse,
        # so turning back immediately would charge into it again
        # (reverse→recover→re-collision loop). Whether the front is clear or not,
        # always drive forward with avoidance steering (STEER) for a while to clear
        # the side of the obstacle, then go through the STEER-release condition
        # (HOLD_MIN + front clear / HOLD_MAX exceeded) and only then move to RECOVER.
        # If the front is empty, steer 0° (straight).
        if self.state == State.BUG_DRIVE:
            self.avoid_phase = Avoid.STEER
            self.avoid_ang   = self.get_avoid_steer_from_sonar()
            self.avoid_t0    = rospy.Time.now()
            rospy.loginfo('[Reverse] done → avoid forward %d° (front=%.0fcm, heading recovery after STEER release)',
                          self.avoid_ang, front_min(self.sonar))
        else:
            rospy.loginfo('[Reverse] done')

    # ══════════════════════════════════════════════════════════════════════
    # UTURN — IMU K-turn  ( (steer) reverse → forward → (steer) reverse … )
    # ══════════════════════════════════════════════════════════════════════
    def execute_uturn(self):
        if self.imu_ready:
            self._uturn_kturn()
        else:
            rospy.logwarn('[UTURN] no IMU → time-based fallback')
            self._uturn_timed()

        self.stop_motor()
        # The U-turn reference heading is single-use → cleared regardless of
        # completion/abort (re-stored on the next approach).
        self.uturn_ref_yaw = None
        # Resume driving only on a clean completion (not aborted by STOP etc.)
        if self.state == State.UTURN:
            rospy.loginfo('[UTURN] done → BUG_DRIVE')
            # [SIM-ONLY] Lock the return to the opposite of the start heading (global +180°).
            #   Real: return_yaw_lock=False → skip below (existing free return).
            if self.return_yaw_lock and self.imu_ready and self.start_yaw is not None:
                self.return_yaw_target = self.start_yaw + 180.0
                self.return_pid.reset(); self._return_prev_t = None
                rospy.loginfo('[UTURN] return yaw locked = %.1f° (start %.1f°+180)',
                              (self.return_yaw_target + 180) % 360 - 180, self.start_yaw)
            self.set_state(State.BUG_DRIVE)

    def _uturn_kturn(self):
        """Alternate reverse(steer) → forward(opposite steer) segments until the
        accumulated IMU yaw reaches UTURN_TARGET_DEG (clockwise / right turn)."""
        # Reference heading: *before* marker approach (original driving direction).
        # Rotating relative to this value (not the heading skewed by the approach)
        # ends up exactly opposite the original direction of travel.
        start = self.uturn_ref_yaw if self.uturn_ref_yaw is not None else self.yaw_deg
        rospy.loginfo('[UTURN/IMU] K-turn start (target %.0f°, ref %.1f°, current %.1f°)',
                      cfg.UTURN_TARGET_DEG, start, self.yaw_deg)
        rate  = rospy.Rate(20)
        # Right-turn direction: reverse=left steer(-90), forward=right steer(+90) → same rotation sense
        seg_pattern = [(-1, -cfg.AVOID_FULL_ANG), (+1, +cfg.AVOID_FULL_ANG)]

        for seg_idx in range(cfg.UTURN_MAX_SEG):
            if self.state != State.UTURN:
                return
            total = yaw_diff(start, self.yaw_deg)
            if total >= cfg.UTURN_TARGET_DEG:
                break

            direction, steer = seg_pattern[seg_idx % 2]
            kind = 'reverse' if direction < 0 else 'forward'
            rospy.loginfo('[UTURN/IMU] Seg%d: %s steer=%d° (accum %.1f°)',
                          seg_idx + 1, kind, steer, total)

            if direction < 0:
                self._esc_reverse_init()

            seg_start = self.yaw_deg
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and self.state == State.UTURN:
                if (rospy.Time.now() - t0).to_sec() > cfg.UTURN_SEG_TO:
                    break
                if yaw_diff(start, self.yaw_deg) >= cfg.UTURN_TARGET_DEG:
                    break
                if yaw_diff(seg_start, self.yaw_deg) >= cfg.UTURN_SEG_DEG:
                    break
                # obstacle check in the direction of travel
                if direction < 0 and rear_min(self.sonar) < cfg.SONAR_REVERSE:
                    rospy.logwarn('[UTURN/IMU] rear blocked → end segment')
                    break
                if direction > 0 and self.is_front_blocked():
                    rospy.logwarn('[UTURN/IMU] front blocked → end segment')
                    break

                self.drive(direction * cfg.SPEED_DRIVE, steer)
                rate.sleep()

            self.stop_motor()
            rospy.sleep(0.3)

        rospy.loginfo('[UTURN/IMU] accumulated rotation %.1f°', yaw_diff(start, self.yaw_deg))

    def _uturn_timed(self):
        """Time-based fallback when there is no IMU (reverse → forward → reverse)."""
        rate = rospy.Rate(20)
        for idx, (direction, steer, dur) in enumerate(cfg.UTURN_TIMED_STEPS):
            if self.state != State.UTURN:
                return
            rospy.loginfo('[UTURN/TIME] Step%d: dir=%d steer=%d dur=%.1fs',
                          idx + 1, direction, steer, dur)
            if direction < 0:
                self._esc_reverse_init()

            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and self.state == State.UTURN:
                if (rospy.Time.now() - t0).to_sec() >= dur:
                    break
                if direction > 0 and self.is_front_blocked():
                    rospy.logwarn('[UTURN/TIME] Step%d front blocked', idx + 1)
                    break
                if direction < 0 and rear_min(self.sonar) < cfg.SONAR_REVERSE:
                    rospy.logwarn('[UTURN/TIME] Step%d rear blocked', idx + 1)
                    break
                self.drive(direction * cfg.SPEED_DRIVE, steer)
                rate.sleep()

            self.stop_motor()
            rospy.sleep(0.2)

    # ══════════════════════════════════════════════════════════════════════
    # MANUAL DRIVE (coordinate-based dead-reckoning)
    # ══════════════════════════════════════════════════════════════════════
    def execute_manual_drive(self):
        goal = self.manual_goal
        if goal is None or self.manual_done:
            self.stop_motor()
            return

        x, y = goal.x, goal.y
        rospy.loginfo('[Manual] goal: x=%.1f y=%.1f cm', x, y)

        # Step 1: rotate to align
        if abs(y) > 2.0:
            angle_deg = clip(math.degrees(math.atan2(y, max(abs(x), 1.0))),
                             -cfg.ANGLE_MAX, cfg.ANGLE_MAX)
            turn_time = abs(angle_deg) * cfg.CM_PER_DEG_TURN
            t0   = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < turn_time and not rospy.is_shutdown():
                if self.state != State.MANUAL_DRIVE or self.is_front_blocked():
                    self.stop_motor(); self.manual_done = True; return
                self.drive(cfg.SPEED_MANUAL, int(angle_deg))
                rate.sleep()
            self.drive(cfg.SPEED_MANUAL, 0)
            rospy.sleep(0.1)

        # Step 2: go straight
        dist = math.hypot(x, y)
        if dist > 2.0:
            drive_time = dist / cfg.CM_PER_SEC_FWD
            spd  = cfg.SPEED_MANUAL if x >= 0 else -cfg.SPEED_MANUAL
            t0   = time.time()
            rate = rospy.Rate(20)
            while time.time() - t0 < drive_time and not rospy.is_shutdown():
                if self.state != State.MANUAL_DRIVE:
                    self.stop_motor(); self.manual_done = True; return
                if self.is_front_blocked() and spd > 0:
                    self.stop_motor(); self.manual_done = True; return
                self.drive(spd, 0)
                rate.sleep()

        self.stop_motor()
        rospy.loginfo('[Manual] reached (estimated)')
        self.manual_done = True
        self.manual_goal = None

    # ══════════════════════════════════════════════════════════════════════
    # BUG_DRIVE — VFH following + avoidance sub-FSM
    # ══════════════════════════════════════════════════════════════════════
    def get_avoid_steer_from_sonar(self):
        """Avoidance steering based on the front sonar (drive forward away from the obstacle)."""
        front_l = self.sonar[cfg.SONAR_FRONT_L]   # front-left (-45°)
        front_c = self.sonar[cfg.SONAR_FRONT]     # front ( 0°)
        front_r = self.sonar[cfg.SONAR_FRONT_R]   # front-right (+45°)

        valid = {}
        if 0 < front_l < 200: valid['left']   = front_l
        if 0 < front_c < 200: valid['center'] = front_c
        if 0 < front_r < 200: valid['right']  = front_r
        if not valid:
            return 0

        closest  = min(valid, key=valid.get)
        min_dist = valid[closest]

        if closest == 'left':
            rospy.loginfo('[Sonar] front-left(%.0fcm) → right steer(+90°)', min_dist)
            return cfg.AVOID_FULL_ANG
        if closest == 'right':
            rospy.loginfo('[Sonar] front-right(%.0fcm) → left steer(-90°)', min_dist)
            return -cfg.AVOID_FULL_ANG
        # front is closest → toward the more open side
        if valid.get('left', 999) < valid.get('right', 999):
            rospy.loginfo('[Sonar] front(%.0fcm) left more blocked → right steer', min_dist)
            return cfg.AVOID_FULL_ANG
        rospy.loginfo('[Sonar] front(%.0fcm) right more blocked → left steer', min_dist)
        return -cfg.AVOID_FULL_ANG

    def _approach_obstacle(self):
        """During MARKER_APPROACH, decide whether to hand off to BUG_DRIVE avoidance
        when a front obstacle is close. Leave at the lower bound (APPROACH_AVOID_CM=40,
        aligned with the distance at which BUG_DRIVE actually starts steering), re-enter
        at the upper bound (AVOID_CLEAR_CM=55): 40~55cm hysteresis prevents oscillation."""
        front = front_min(self.sonar)
        return 0 < front < cfg.APPROACH_AVOID_CM

    def _bug_drive_step(self):
        now   = rospy.Time.now()
        front = front_min(self.sonar)

        # ── NONE: normal driving, decide whether to enter avoidance ──────────────
        if self.avoid_phase == Avoid.NONE:
            sonar_close = front < cfg.SONAR_EMERGENCY * 1.5
            vfh_sharp   = abs(self.vfh_angle) >= cfg.AVOID_TRIG_ANG
            if sonar_close or vfh_sharp:
                self.avoid_phase = Avoid.STEER
                self.avoid_ang   = (self.get_avoid_steer_from_sonar() if sonar_close
                                    else (cfg.AVOID_FULL_ANG if self.vfh_angle > 0
                                          else -cfg.AVOID_FULL_ANG))
                self.avoid_t0 = now
                if self.imu_ready and self.heading_target is None:
                    self.heading_target = self.yaw_deg   # store original heading on entry
                rospy.loginfo('[Avoid] enter (steer=%d°, head0=%s)',
                              self.avoid_ang,
                              '%.1f' % self.heading_target if self.heading_target is not None else 'NA')
            else:
                # Capture the global start heading on the first BUG_DRIVE entry (return-yaw ref)
                if self.start_yaw is None and self.imu_ready:
                    self.start_yaw = self.yaw_deg
                # [SIM-ONLY] After a U-turn, hold the return at a fixed global yaw
                #   (PID heading-hold instead of free vfh driving).
                #   Real: return_yaw_target=None (lock off) → else branch follows vfh.
                if (self.return_yaw_lock and self.return_yaw_target is not None
                        and self.imu_ready):
                    err = yaw_signed_diff(self.return_yaw_target, self.yaw_deg)
                    dt  = self._dt('_return_prev_t')
                    steer = int(clip(-self.return_pid.step(err, dt),
                                     -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
                    self.drive(self.vfh_speed, steer)
                else:
                    self.drive(self.vfh_speed, self.vfh_angle)
                return

        # ── STEER: hold avoidance steering ───────────────────────────────────────
        if self.avoid_phase == Avoid.STEER:
            self.drive(self.vfh_speed, self.avoid_ang)
            held = (now - self.avoid_t0).to_sec()
            # Release conditions: (a) front sufficiently clear, or (b) max hold exceeded.
            # (b) handles passing alongside a box where front never rises to CLEAR:
            # straighten the wheels and recover to a straight heading, preventing
            # "driving forward with the wheels cranked".
            cleared  = front >= cfg.AVOID_CLEAR_CM
            timed_out = held >= cfg.AVOID_HOLD_MAX
            if held >= cfg.AVOID_HOLD_MIN and (cleared or timed_out):
                why = 'front=%.0fcm' % front if cleared else 'hold>%.1fs' % cfg.AVOID_HOLD_MAX
                if self.imu_ready and self.heading_target is not None:
                    self.avoid_phase = Avoid.RECOVER
                    self.recover_t0  = now
                    self.recover_pid.reset()
                    self._recover_prev_t = None
                    rospy.loginfo('[Avoid] release steering → heading recovery (%s)', why)
                else:
                    self._reset_avoid()
                    rospy.loginfo('[Avoid] release (no IMU, %s)', why)
            return

        # ── RECOVER: return to the original heading (PID) ────────────────────────
        if self.avoid_phase == Avoid.RECOVER:
            # obstacle again during recovery → back to avoidance
            if front < cfg.SONAR_EMERGENCY * 1.5:
                self.avoid_phase = Avoid.STEER
                self.avoid_ang   = self.get_avoid_steer_from_sonar()
                self.avoid_t0    = now
                rospy.loginfo('[Recover] obstacle re-detected → back to avoidance')
                return

            err = yaw_signed_diff(self.heading_target, self.yaw_deg)
            if abs(err) <= cfg.RECOVER_TOL_DEG or \
               (now - self.recover_t0).to_sec() > cfg.RECOVER_TIMEOUT:
                rospy.loginfo('[Recover] done (error %.1f°)', err)
                self._reset_avoid()
                self.drive(self.vfh_speed, self.vfh_angle)
                return

            # err>0 → yaw must increase (turn left, negative steering). Negate the PID output.
            dt = self._dt('_recover_prev_t')
            steer = int(clip(-self.recover_pid.step(err, dt), -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
            self.drive(self.vfh_speed, steer)
            return

    # ══════════════════════════════════════════════════════════════════════
    # Main loop
    # ══════════════════════════════════════════════════════════════════════
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.state in (State.IDLE, State.STOP):
                self.stop_motor(); rate.sleep(); continue

            if self.state == State.MANUAL_DRIVE:
                self.execute_manual_drive(); rate.sleep(); continue

            if self.state == State.UTURN:
                self.execute_uturn(); continue

            # ── BUG_DRIVE / MARKER_APPROACH common: emergency reverse ────────────
            if self.reversing:
                rate.sleep(); continue
            if self.is_front_blocked():
                rospy.logwarn_throttle(1, '[Main] EMERGENCY → reverse')
                if self.imu_ready and self.heading_target is None:
                    self.heading_target = self.yaw_deg   # store heading before reverse
                t = threading.Thread(target=self.reverse_motor,
                                     kwargs={'speed': cfg.SPEED_DRIVE, 'target_front_cm': 30})
                t.daemon = True
                t.start()
                rate.sleep(); continue

            if self.state == State.MARKER_APPROACH:
                if self.marker_visible():
                    if self.marker_pixel_w >= cfg.MARKER_CLOSE_PX:
                        rospy.loginfo('[Main] marker approach complete(%.0fpx) → UTURN', self.marker_pixel_w)
                        self.stop_motor()
                        self.set_state(State.UTURN)
                    elif self._approach_obstacle():
                        # Even if the marker is visible, a close front obstacle stops
                        # marker following and returns to BUG_DRIVE to handle it with
                        # the (proven) avoidance sub-FSM. Once the obstacle is cleared,
                        # BUG_DRIVE hands back to MARKER_APPROACH.
                        rospy.loginfo_throttle(2, '[Approach] front obstacle → switch to BUG_DRIVE avoidance')
                        self.set_state(State.BUG_DRIVE)
                    else:
                        # bearing(rad) error → PID steering (right marker = +bearing = right steer)
                        dt = self._dt('_marker_prev_t')
                        steer = int(clip(self.marker_pid.step(self.marker_bearing, dt),
                                         -cfg.ANGLE_MAX, cfg.ANGLE_MAX))
                        self.drive(cfg.SPEED_DRIVE, steer)
                else:
                    self.drive(cfg.SPEED_DRIVE, 0)
                rate.sleep(); continue

            if self.state == State.BUG_DRIVE:
                if self.marker_visible():
                    self.marker_seen += 1
                else:
                    self.marker_seen = 0
                # Switch to approach only when the front is clear (avoid first if blocked).
                if (self.marker_seen >= cfg.MARKER_DEBOUNCE
                        and self.avoid_phase == Avoid.NONE
                        and front_min(self.sonar) > cfg.AVOID_CLEAR_CM):
                    rospy.loginfo('[Main] marker → MARKER_APPROACH')
                    self.marker_seen = 0
                    # Store the heading *before* approach as the U-turn reference (first time only).
                    # Even if it toggles to avoidance during approach and re-enters,
                    # the original driving direction is preserved.
                    if self.uturn_ref_yaw is None and self.imu_ready:
                        self.uturn_ref_yaw = self.yaw_deg
                        rospy.loginfo('[Main] stored U-turn reference heading: %.1f°', self.uturn_ref_yaw)
                    self.set_state(State.MARKER_APPROACH)
                    continue
                self._bug_drive_step()

            rate.sleep()


if __name__ == '__main__':
    try:
        RodongMain().run()
    except rospy.ROSInterruptException:
        pass
