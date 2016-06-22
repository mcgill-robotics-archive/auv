cmake_minimum_required(VERSION 2.8.3)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
)

###################
## Action config ##
###################

 add_action_files(
   DIRECTORY action
   FILES
   initialize.action
   move.action
 )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

###################
## Catkin config ##
###################

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
)
