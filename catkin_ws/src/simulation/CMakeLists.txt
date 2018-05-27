cmake_minimum_required(VERSION 2.8.3)
project(simulation)

# Catkin Dependencies
find_package(catkin REQUIRED
  auv_msgs
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  robot_state_publisher
  rospy
  std_msgs
  tf
  urdf
  xacro
)

# System Dependencies
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# None yet...

###################
## Catkin Config ##
###################

catkin_package(
  CATKIN_DEPENDS
  auv_msgs
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  robot_state_publisher
  rospy
  tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Thruster Plugin
add_library(
  thruster_plugin
  src/gazebo/thruster_plugin.cpp
)
target_link_libraries(
  thruster_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(
  thruster_plugin
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Depth Plugin
add_library(
  depth_plugin
  src/gazebo/depth_plugin.cpp
)
target_link_libraries(
  depth_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(
  depth_plugin
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


#############
## Install ##
#############

# None yet...

#############
## Testing ##
#############

# None yet...
