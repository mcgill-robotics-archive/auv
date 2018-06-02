cmake_minimum_required(VERSION 2.8.3)
project(cv)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CvTarget.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  pointgrey_camera_driver
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
