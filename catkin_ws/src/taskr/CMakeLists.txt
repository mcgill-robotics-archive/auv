cmake_minimum_required(VERSION 2.8.3)
project(taskr)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  rospy
  std_msgs
)

###################
## Catkin config ##
###################

catkin_package(
  CATKIN_DEPENDS
  actionlib
  rospy std_msgs
)
