cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  auv_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
)

catkin_python_setup()


add_action_files(DIRECTORY
    action FILES
        GoToDepth.action
)

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    auv_msgs
    geometry_msgs
    actionlib_msgs
    actionlib
)

add_subdirectory(test)
