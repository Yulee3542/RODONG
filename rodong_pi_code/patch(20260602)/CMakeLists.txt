cmake_minimum_required(VERSION 3.0.2)
project(rodong)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  xycar_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rodong_main.py
  scripts/rodong_teleop.py
  scripts/vfh_planner.py
  scripts/aruco_detector.py
  scripts/yolo_node.py
  scripts/line_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
