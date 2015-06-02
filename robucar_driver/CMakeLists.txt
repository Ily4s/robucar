cmake_minimum_required(VERSION 2.8.3)
project(robucar_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  RobotData.msg
  RobotDataStamped.msg
  RobucarDrive.msg
  RobucarDriveStamped.msg
  JoyDrive.msg
  JoyDriveStamped.msg
  SimpleRobotData.msg
)

add_service_files(
  FILES
  DriveMode.srv
  RobotPTU.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)