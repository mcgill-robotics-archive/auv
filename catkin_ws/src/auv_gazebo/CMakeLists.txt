cmake_minimum_required(VERSION 2.8.3)
project(auv_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  robot_state_publisher
  rospy
  sensor_msgs
  std_msgs
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
