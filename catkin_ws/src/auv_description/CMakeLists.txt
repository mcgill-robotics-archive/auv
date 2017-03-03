cmake_minimum_required(VERSION 2.8.3)
project(auv_description)

find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  robot_state_publisher
  rospy
  std_msgs
  urdf
  xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
