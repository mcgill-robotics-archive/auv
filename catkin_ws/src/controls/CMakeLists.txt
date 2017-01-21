cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  auv_msgs
  geometry_msgs
  message_generation
)

add_message_files(
    FILES
    Pose.msg
)

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS 
    message_runtime)
