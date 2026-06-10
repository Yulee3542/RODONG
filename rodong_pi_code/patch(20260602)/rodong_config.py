# -*- coding: utf-8 -*-
"""
rodong_config.py — RODONG shared configuration (single source of tuning constants)
================================================================
Every node (vfh_planner / rodong_main / aruco_detector ...) imports its tuning
constants from here. The module has no ROS dependency, so it can be imported
anywhere and unit-tested easily.

Previously the same values (speeds / thresholds / sonar beam mapping / ArUco id ...)
were duplicated in vfh_planner.py and rodong_main.py and could drift apart — they
are unified here.
"""

# ── Speed [motor units] ────────────────────────────────────────────
# ★★ Single source of truth for speed: change this ONE line (SPEED) and every
#    driving speed follows. (forward / avoid / approach / U-turn / MANUAL / reverse
#    are all derived from SPEED below.)
SPEED = 30             # ← edit here only
# The motor does not turn enough at low speed (ESC deadband), so every situation
# uses the same unified speed.

SPEED_DRIVE  = SPEED   # normal / avoid / approach / U-turn driving speed
SPEED_MANUAL = SPEED   # MANUAL mode speed
SPEED_BACK   = -SPEED  # reverse when fully blocked (vfh_planner)

# ── Steering [deg] ─────────────────────────────────────────────────
ANGLE_MAX      = 90    # physical max steering
AVOID_FULL_ANG = 90    # full avoidance steering

# ── Sonar beam index / angle [deg] (memorized mapping) ─────────────
#   idx0=left(-90)  1=front-left(-45)  2=front-right(+45)  3=front(0)
#   idx4=right(+90) 5=rear-right(+135) 6=rear(180)         7=rear-left(-135)
BEAM_ANGLES   = [-90, -45, 45, 0, 90, 135, 180, -135]
SONAR_LEFT    = 0
SONAR_FRONT_L = 1
SONAR_FRONT_R = 2
SONAR_FRONT   = 3
SONAR_RIGHT   = 4
SONAR_REAR_R  = 5
SONAR_REAR    = 6
SONAR_REAR_L  = 7
FRONT_IDXS = (SONAR_FRONT_L, SONAR_FRONT_R, SONAR_FRONT)   # (1, 2, 3)
REAR_IDXS  = (SONAR_REAR_R, SONAR_REAR, SONAR_REAR_L)      # (5, 6, 7)

# ── Sonar distance thresholds [cm] ─────────────────────────────────
THRESHOLD       = 40.0   # vfh: at/below this distance → treated as an obstacle
SLOW_DIST       = 50.0   # vfh: at/below this distance → slow down
EMERGENCY       = 12.0   # vfh: front emergency (reference value)
SONAR_EMERGENCY = 15     # main: front emergency (at/below → reverse)
SONAR_REVERSE   = 15     # main: rear-obstacle detection distance while reversing

# ── VFH histogram ──────────────────────────────────────────────────
N_SECTORS    = 7
SECTOR_DEG   = 30.0
SECTOR_ANGLE = [-90, -90, -90, 0, 90, 90, 90]   # sector center → steering angle
OPEN_THRESH  = 0.5       # hist at/below this → sector considered passable
W_GOAL    = 1.0          # prefer goal direction
W_HEADING = 0.4          # keep going straight (anti-zigzag)
W_SMOOTH  = 0.2          # keep previous direction (anti-jitter)

# ── Timeouts [s] ───────────────────────────────────────────────────
GOAL_TIMEOUT   = 1.5
YOLO_TIMEOUT   = 1.0
BOUND_TIMEOUT  = 0.7     # validity window for /rodong/boundary
MARKER_TIMEOUT = 0.5     # /aruco_pose freshness (marker decision in main)

# ── Floor boundary line (vfh) ──────────────────────────────────────
BOUND_TH      = 0.10     # left/center/right zone threshold
BOUND_NEAR_TH = 0.15     # near (right in front of the car) threshold

# ── YOLO ───────────────────────────────────────────────────────────
CLS_CLIMB  = 0
CLS_AVOID  = 1
CLS_IGNORE = 2
CLIMB_BOTTOM_RATIO = 0.82
USE_CLIMB = False        # model accuracy too low → CLIMB decision disabled

# ── Avoidance sub-FSM (main) ───────────────────────────────────────
AVOID_TRIG_ANG = 32      # if |vfh_angle| >= this → enter avoidance sub-FSM (±full-steer latch).
                         # the vfh histogram only emits -90/0/+90, so hardware behavior is
                         # unchanged (±90 > 32). vision proportional steering (<=30°) stays
                         # below this → smooth following without latching.
AVOID_HOLD_MIN = 1.5     # minimum hold time for avoidance steering (s)
AVOID_HOLD_MAX = 3.5     # maximum hold time for avoidance steering (s). Even if the front
                         # never clears (front < CLEAR), after this time force a transition
                         # to heading recovery to straighten the wheels.
                         # (prevents "driving forward with the wheels cranked indefinitely")
AVOID_CLEAR_CM = 55      # front >= this → release avoidance / re-enter approach (hysteresis upper)
APPROACH_AVOID_CM = 40   # during approach, front < this → switch to avoidance (hysteresis lower)

# ── Heading recovery (main, RECOVER) ───────────────────────────────
RECOVER_TOL_DEG = 8.0    # within this error → recovery complete
RECOVER_TIMEOUT = 4.0    # recovery phase timeout (s)

# ── Marker (main) ──────────────────────────────────────────────────
TARGET_ID       = 1
MARKER_CLOSE_PX = 80     # marker pixel width >= this → approach complete
MARKER_DEBOUNCE = 5      # consecutive detection frames

# ── MANUAL dead-reckoning ──────────────────────────────────────────
CM_PER_SEC_FWD  = 15.0
CM_PER_DEG_TURN = 0.12

# ── U-turn (IMU K-turn) ────────────────────────────────────────────
UTURN_TARGET_DEG = 170.0
UTURN_SEG_DEG    = 60.0
UTURN_SEG_TO     = 4.0
UTURN_MAX_SEG    = 5
# Time-based fallback when no IMU: (direction(+fwd/-rev), steering, duration s)
UTURN_TIMED_STEPS = [(-1, -90, 2.0), (+1, 90, 2.0), (-1, -90, 2.0)]

# ── PID gains ──────────────────────────────────────────────────────
# Marker approach: bearing[rad] error → steering[deg]. (was: pixel error × 0.47 simple P)
#   bearing≈0.5rad (screen edge) → kp*0.5 ≈ 60° → strong steering at full offset.
MARKER_PID  = dict(kp=120.0, ki=0.0, kd=15.0, out_limit=ANGLE_MAX)
# Heading recovery: yaw error[deg] → steering[deg]. (was: -3.0×error simple P; kd added for damping)
RECOVER_PID = dict(kp=3.0, ki=0.0, kd=0.4, out_limit=ANGLE_MAX)
