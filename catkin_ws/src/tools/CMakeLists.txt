cmake_minimum_required(VERSION 2.8.3)
project(tools)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  rospy
  sensor_msgs
  std_msgs
)

# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## None for now...

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  auv_msgs
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

## None for now...

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/drytest/full_test.py
  scripts/drytest/sensors_test.py
  scripts/drytest/thrusters_test.py
  scripts/drytest/camera_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## None for now...
