cmake_minimum_required(VERSION 2.8.3)
project(auv_gazebo)

find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  robot_state_publisher
  rospy
  tf
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  auv_msgs
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  robot_state_publisher
  rospy
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(thruster_plugin src/thruster_plugin.cpp)
target_link_libraries(thruster_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(thruster_plugin ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
