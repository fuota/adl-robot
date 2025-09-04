cmake_minimum_required(VERSION 3.0.2)
project(adl_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  ros_nodes/task_executor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
