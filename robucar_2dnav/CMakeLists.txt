cmake_minimum_required(VERSION 2.8.3)
project(robucar_2dnav)

find_package(catkin REQUIRED COMPONENTS
  move_base
  robucar_controller
  robucar_description
  robucar_driver
  robucar_tele
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)