# ROS1 Noetic + Gazebo 11 — RODONG FSM/회피 로직 검증용 ROS1 컨테이너.
# RODONG 은 ROS1(rospy) 코드라 별도 ROS2 트윈과 섞지 않고 여기서 그대로 돌린다.
# Ubuntu 20.04(컨테이너 내부)로 고정 — 호스트가 Ubuntu 26.04 라도 재현 가능.
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# gazebo_ros_pkgs / gazebo_plugins (planar_move·ray laser·imu 플러그인 .so),
# catkin 빌드 도구, headless 소프트웨어-GL 렌더링(xvfb)용 패키지.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-gazebo-ros \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-plugins \
        ros-noetic-xacro \
        ros-noetic-tf \
        python3-catkin-tools \
        python3-rospkg \
        python3-numpy \
        python3-opencv \
        ros-noetic-cv-bridge \
        python3-pip \
        xvfb mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# YOLO 추론 엔진 (yolo_node.py 가 onnxruntime 사용). cv2/cv_bridge 는 desktop-full 포함.
RUN pip3 install --no-cache-dir onnxruntime==1.16.3

# yolo_node 의 하드코딩 모델경로 → 레포의 onnx 로 심볼릭 링크.
# (/workspace 는 런타임 마운트라 링크 대상은 실행 시점에 해석된다.)
RUN mkdir -p /root/xycar_ws/src/rodong/models \
 && ln -s /workspace/rodong_yolo/rodong.onnx /root/xycar_ws/src/rodong/models/rodong.onnx

# 헤드리스 박스(GPU 없음)에서 CPU ray 라이다는 GL 없이도 동작하지만,
# 혹시 모를 렌더 경로를 위해 소프트웨어 GL 을 기본값으로 둔다(SOFTWARE_GL 로 토글).
ENV LIBGL_ALWAYS_SOFTWARE=1

# 모든 셸에서 ROS + 워크스페이스 오버레이 자동 source + Gazebo 모델 경로.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo '[ -f /workspace/catkin_ws/devel/setup.bash ] && source /workspace/catkin_ws/devel/setup.bash' >> /root/.bashrc \
 && echo 'export GAZEBO_MODEL_PATH=/workspace/rodong_sim/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}' >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
