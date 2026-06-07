# rodong_sim — 단순 박스 모델 Gazebo 시뮬레이션

Xycar 하드웨어/시뮬 패키지 없이 **RODONG FSM·회피 로직(`rodong_main.py` + `vfh_planner.py`)**
이 동작하는지 확인하기 위한 최소 시뮬레이션이다. 실제 Xycar 모델이 아니라
**토픽 인터페이스만 동일하게** 맞춘 박스 차량을 쓴다.

```
Gazebo(박스+라이다+IMU) ──/scan──┐                 ┌──/xycar_ultrasonic──► vfh_planner ──/rodong/vfh_cmd──► rodong_main
                                 ├─ sim_bridge ────┤                                                          │
            planar_move ◄──/cmd_vel──┘             └◄────────────────── /xycar_motor ◄──────────────────────┘
```

- `/scan`(360° 라이다) → 8빔 각도 `[-90,-45,45,0,90,135,180,-135]°` 샘플링 → `/xycar_ultrasonic`(cm)
- `/xycar_motor`(speed, angle) → 자전거 모델로 `(v, yaw_rate)` 변환 → `/cmd_vel`
- IMU → `/imu/data` (헤딩 복귀 RECOVER 단계 입력)
- 카메라/YOLO/ArUco 는 생략 → 마커가 안 보여 `rodong_main` 은 BUG_DRIVE 회피만 수행

> ⚠️ 이 패키지는 ROS/Gazebo 가 없는 머신에서 작성·검증되었다(문법/XML well-formed 까지만 확인).
> 실제 Gazebo 구동은 아래 환경에서 처음 돌려보며 플러그인 이름/게인을 미세조정해야 할 수 있다.

## 요구 환경
- ROS1 (Noetic 권장; Melodic 도 대체로 동작)
- `gazebo_ros`, `gazebo_plugins` (`libgazebo_ros_planar_move`, `libgazebo_ros_laser`, `libgazebo_ros_imu_sensor`)
- Python: `rospy`, `tf`

이 개발 박스엔 ROS1 이 직접 설치돼 있지 않으므로 **Docker(ROS1 Noetic + Gazebo 11)** 로 돌린다.
(`docker/`, `scripts/sim_*.sh` 는 레포 루트에 있다.)

## 빌드 & 실행 — Docker (권장, 헤드리스)
```bash
# 1) 이미지 + catkin 워크스페이스 빌드
scripts/sim_build.sh
# 2) 시뮬 기동 (헤드리스 = gzserver만; 'gui' 인자로 Gazebo 창)
scripts/sim_run.sh            # 또는: scripts/sim_run.sh gui
# 3) 다른 터미널에서 주행 시작 (IDLE → BUG_DRIVE)
scripts/sim_cmd.sh INIT       # STOP / UTURN / MANUAL 도 동일
```
- `sim_build.sh` 가 `catkin_ws/src` 에 `rodong_sim`·`xycar_msgs` 심볼릭 링크를 만들고
  컨테이너 안에서 `catkin_make` 한다. 빌드 산출물은 호스트 `catkin_ws/` 에 남는다.
- 헤드리스에선 CPU ray 라이다라 GPU 없이 동작하며, 렌더 경로용으로 `xvfb` 를 끼워 실행한다.

## 빌드 & 실행 — 직접 설치된 ROS1 이 있을 때
```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
ln -s /home/yulee23/RODONG/rodong_sim .
ln -s /home/yulee23/RODONG/xycar_msgs .          # 이미 있으면 생략
cd ~/catkin_ws && catkin_make && source devel/setup.bash
roslaunch rodong_sim rodong_sim.launch app_dir:=/home/yulee23/RODONG/rodong_pi_code/patch\(20260602\)
# 다른 터미널:
rostopic pub -1 /rodong/cmd std_msgs/String "data: 'INIT'"
```

## 인지 테스트 — 카메라 + ArUco + YOLO
```bash
scripts/sim_run.sh perception      # 카메라 월드(정면 ArUco id=1 패널) + aruco_detector + yolo_node
scripts/sim_cmd.sh INIT            # 마커로 접근 → MARKER_APPROACH → UTURN
```
- 박스 모델에 카메라(`/usb_cam/image_raw`, 640x480@15Hz, 헤드리스선 software GL로 ~10Hz) 추가.
- `worlds/perception.world` 정면 1.3m 에 `models/aruco_marker`(DICT_4X4_50, id=1, 0.15m).
- **ArUco**: 실제 검출 동작 — `/aruco_pose` 발행(거리/bearing/pixel_w) → `rodong_main`
  `BUG_DRIVE → MARKER_APPROACH →`(pixel_w≥`MARKER_CLOSE_PX`)`→ UTURN(K-turn)`. (검증됨)
- **YOLO**: 노드 통합만 — `onnxruntime`으로 `rodong.onnx` 로드 후 카메라 프레임 추론(무에러).
  모델이 실사 계단/경사로(`CLIMB`/`AVOID`) detector라 **Gazebo 프리미티브는 검출 안 됨**.
  다운스트림 회피 로직을 보려면 `/rodong/yolo` 를 직접 발행해 주입(`std_msgs/Float32MultiArray`,
  `[cls, conf, cx_norm, cy_norm, bottom_ratio]`).

## 무엇을 확인할 수 있나
- **BUG_DRIVE 회피**: 박스 장애물 앞에서 VFH 섹터 선택 → 조향 회피 → 통과
- **비상 후진**: 정면 막다른 벽(`wall_front`)에서 `SONAR_EMERGENCY` 이내 → `reverse_motor`
- **후진 후 직진(이번 수정)**: 후진 종료 시 `drive(0,0)` 로 바퀴를 펴고 잠깐 정지한 뒤
  전진 재개 → 바퀴가 꺾인 채 출발하지 않는지 확인
- **RECOVER**: IMU yaw 로 원래 헤딩 복귀
- **인지 파이프라인**: 카메라 → ArUco 검출 → 마커 접근 → IMU K-turn (위 "인지 테스트")

로그 관찰:
```bash
rostopic echo /xycar_ultrasonic     # 8빔 cm
rostopic echo /xycar_motor          # FSM 출력 (speed, angle)
rostopic echo /cmd_vel              # 브리지 변환 결과
```

## 튜닝 (`sim_bridge` 파라미터)
| param | 기본 | 의미 |
|---|---|---|
| `speed_to_ms` | 0.03 | 모터단위→m/s (25 → 0.75 m/s) |
| `wheelbase` | 0.30 | 자전거 모델 L [m]. 작을수록 급선회 |
| `max_steer_deg` | 60 | tan 발산 방지용 조향 클램프 |
| `max_cm` | 300 | 라이다 무반사 시 보고 거리 |

## 한계 / 주의
- `planar_move` 는 **기구학적**(힘/마찰 없음) 유니사이클이다. 실차의 ESC 데드밴드·서보
  기계 지연은 재현되지 않으므로, "후진 직진" 수정의 *물리적* 효과보다는 **명령 시퀀스
  로직**(후진→`drive(0,0)`→정지→전진) 검증에 적합하다.
- 부호 규약: Xycar `+angle=우회전` → `/cmd_vel` 음의 yaw(시계방향, 실차와 동일). 만약
  Gazebo 에서 회피가 반대로 돌면 `sim_bridge.cb_motor` 의 `yaw_rate` 부호를 뒤집으면 된다.
- 플러그인 `.so` 이름은 배포판마다 다를 수 있다(예: GPU 라이다 `libgazebo_ros_gpu_laser`).
  로드 실패 시 설치된 `gazebo_plugins` 의 실제 이름으로 교체.
