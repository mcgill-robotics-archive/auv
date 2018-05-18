cmake_minimum_required(VERSION 2.8.3)
project(cv)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(CATKIN_DEPENDS
  pointgrey_camera_driver
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
