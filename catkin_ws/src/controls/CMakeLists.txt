cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  auv_msgs
  geometry_msgs
)

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
)
