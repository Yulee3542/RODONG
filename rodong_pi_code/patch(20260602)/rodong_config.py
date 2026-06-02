# -*- coding: utf-8 -*-
"""
rodong_config.py — RODONG 공용 설정 (튜닝 상수 단일 출처)
================================================================
모든 노드(vfh_planner / rodong_main / aruco_detector ...)가 공유하는 상수를
한 곳에 모은다. ROS 의존성이 없으므로 어디서나 import 가능하고 단위테스트가 쉽다.

이전에는 같은 값(속도/임계/초음파 빔 매핑/ArUco ID 등)이 vfh_planner.py 와
rodong_main.py 에 따로 정의되어 서로 어긋날 위험이 있었다 → 여기로 통합.
"""

# ── 속도 [모터 단위] ───────────────────────────────────────────────
SPEED_DRIVE  = 25      # 일반/회피/접근/유턴 공통 주행 속도
SPEED_MANUAL = 20      # MANUAL 모드 속도
SPEED_BACK   = -25     # 전방향 막힘 시 후진 (vfh_planner)

# ── 조향 [deg] ─────────────────────────────────────────────────────
ANGLE_MAX      = 90    # 물리 최대 조향
AVOID_FULL_ANG = 90    # 회피 풀조향

# ── 초음파 빔 인덱스 / 각도 [deg] (메모리 매핑) ────────────────────
#   idx0=좌(-90)  1=좌전(-45)  2=우전(+45)  3=전(0)
#   idx4=우(+90)  5=우후(+135) 6=후(180)    7=좌후(-135)
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

# ── 초음파 거리 임계 [cm] ──────────────────────────────────────────
THRESHOLD       = 40.0   # vfh: 이 거리 이하 → 장애물 간주
SLOW_DIST       = 50.0   # vfh: 이 거리 이하 → 감속
EMERGENCY       = 12.0   # vfh: 정면 비상 (참고값)
SONAR_EMERGENCY = 15     # main: 전방 비상 (이하 → 후진)
SONAR_REVERSE   = 15     # main: 후진 중 후방 장애물 감지 거리

# ── VFH 히스토그램 ─────────────────────────────────────────────────
N_SECTORS    = 7
SECTOR_DEG   = 30.0
SECTOR_ANGLE = [-90, -90, -90, 0, 90, 90, 90]   # 섹터 중심 → 조향각
OPEN_THRESH  = 0.5       # hist 이하면 통행 가능 섹터로 간주
W_GOAL    = 1.0          # goal 방향 선호
W_HEADING = 0.4          # 직진 유지 (지그재그 방지)
W_SMOOTH  = 0.2          # 직전 방향 유지 (떨림 방지)

# ── 타임아웃 [s] ───────────────────────────────────────────────────
GOAL_TIMEOUT   = 1.5
YOLO_TIMEOUT   = 1.0
BOUND_TIMEOUT  = 0.7     # /rodong/boundary 유효 시간
MARKER_TIMEOUT = 0.5     # /aruco_pose 신선도 (main 마커 판정)

# ── 바닥 경계선 (vfh) ──────────────────────────────────────────────
BOUND_TH      = 0.10     # 좌/중/우 구역 임계
BOUND_NEAR_TH = 0.15     # near(차 바로 앞) 임계

# ── YOLO ───────────────────────────────────────────────────────────
CLS_CLIMB  = 0
CLS_AVOID  = 1
CLS_IGNORE = 2
CLIMB_BOTTOM_RATIO = 0.82
USE_CLIMB = False        # 모델 성능 부족 → CLIMB 판정 비활성

# ── 회피 서브-FSM (main) ───────────────────────────────────────────
AVOID_TRIG_ANG = 25      # |vfh_angle| 이 이상이면 회피 진입
AVOID_HOLD_MIN = 1.5     # 회피 조향 최소 유지 시간 (s)
AVOID_CLEAR_CM = 55      # 전방 이 이상이면 회피 해제 조건

# ── 헤딩 복귀 (main, RECOVER) ──────────────────────────────────────
RECOVER_TOL_DEG = 8.0    # 이 오차 이내면 복귀 완료
RECOVER_TIMEOUT = 4.0    # 복귀 단계 타임아웃 (s)

# ── 마커 (main) ────────────────────────────────────────────────────
TARGET_ID       = 1
MARKER_CLOSE_PX = 80     # 마커 픽셀폭 이 이상 → 접근 완료
MARKER_DEBOUNCE = 5      # 연속 검출 프레임 수

# ── MANUAL dead-reckoning ──────────────────────────────────────────
CM_PER_SEC_FWD  = 15.0
CM_PER_DEG_TURN = 0.12

# ── 유턴 (IMU K-turn) ──────────────────────────────────────────────
UTURN_TARGET_DEG = 170.0
UTURN_SEG_DEG    = 60.0
UTURN_SEG_TO     = 4.0
UTURN_MAX_SEG    = 5
# IMU 없을 때 시간 기반 fallback: (방향(+전진/-후진), 조향, 지속시간 s)
UTURN_TIMED_STEPS = [(-1, -90, 2.0), (+1, 90, 2.0), (-1, -90, 2.0)]

# ── PID 게인 ───────────────────────────────────────────────────────
# 마커 접근: bearing[rad] 오차 → 조향[deg]. (이전: 픽셀오차×0.47 단순 P)
#   bearing≈0.5rad(화면끝) → kp*0.5 ≈ 60° → 풀오프셋에서 강한 조향.
MARKER_PID  = dict(kp=120.0, ki=0.0, kd=15.0, out_limit=ANGLE_MAX)
# 헤딩 복귀: yaw 오차[deg] → 조향[deg]. (이전: -3.0×오차 단순 P, kd 추가로 댐핑)
RECOVER_PID = dict(kp=3.0, ki=0.0, kd=0.4, out_limit=ANGLE_MAX)
