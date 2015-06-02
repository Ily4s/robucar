cmake_minimum_required(VERSION 2.8.3)
project(robucar_controller)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)