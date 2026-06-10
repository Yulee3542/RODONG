# ROS1 Noetic + Gazebo 11 — ROS1 container for validating RODONG FSM/avoidance logic.
# RODONG is ROS1 (rospy) code, so run it here as-is without mixing in a separate ROS2 twin.
# Pinned to Ubuntu 20.04 (inside the container) — reproducible even if the host is Ubuntu 26.04.
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# gazebo_ros_pkgs / gazebo_plugins (planar_move / ray laser / imu plugin .so),
# catkin build tools, and packages for headless software-GL rendering (xvfb).
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

# YOLO inference engine (yolo_node.py uses onnxruntime). cv2/cv_bridge come with desktop-full.
RUN pip3 install --no-cache-dir onnxruntime==1.16.3

# Symlink yolo_node's hardcoded model path -> the repo's onnx.
# (/workspace is a runtime mount, so the link target is resolved at run time.)
RUN mkdir -p /root/xycar_ws/src/rodong/models \
 && ln -s /workspace/rodong_yolo/rodong.onnx /root/xycar_ws/src/rodong/models/rodong.onnx

# On a headless box (no GPU), the CPU ray lidar works without GL, but
# keep software GL as the default for any rendering paths just in case (toggle via SOFTWARE_GL).
ENV LIBGL_ALWAYS_SOFTWARE=1

# Auto-source ROS + the workspace overlay in every shell + set the Gazebo model path.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo '[ -f /workspace/catkin_ws/devel/setup.bash ] && source /workspace/catkin_ws/devel/setup.bash' >> /root/.bashrc \
 && echo 'export GAZEBO_MODEL_PATH=/workspace/rodong_sim/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}' >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
