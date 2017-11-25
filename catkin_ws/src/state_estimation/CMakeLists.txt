cmake_minimum_required(VERSION 2.8.3)
project(state_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  message_generation
  nav_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  tf2
  cv_bridge
  rospy
  auv_msgs
  angles
  tf_conversions
  eigen_conversions
)

## Boost requires its own find_package
find_package(Boost REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED)

## Have to put this in a separate find_package because we need to
## find cmake_modules first
find_package(Eigen3 REQUIRED)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  tf
  tf2
  boost
  auv_msgs
  angles
  tf_conversions
  eigen_conversions
  cv_bridge
)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

###########
## Build ##
###########

add_subdirectory(pose)
add_subdirectory(tf)
add_subdirectory(ukf)
add_subdirectory(servo)
add_subdirectory(detectors)
