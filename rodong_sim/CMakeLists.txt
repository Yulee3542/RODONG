cmake_minimum_required(VERSION 3.0.2)
project(rodong_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_bridge.py
  scripts/run_app.py
  scripts/sim_vision_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
