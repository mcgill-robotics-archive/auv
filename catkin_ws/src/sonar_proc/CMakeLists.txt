cmake_minimum_required(VERSION 2.8.3)
project(sonar_proc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  pcl_msgs
  message_generation
)

add_message_files(
    FILES
    Cluster.msg
    ClusterArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

###################
## Catkin config ##
###################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_msgs
    message_generation
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pcl_scan_preprocessor
  src/scan_preprocessor.cpp
)

add_dependencies(pcl_scan_preprocessor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(pcl_scan_preprocessor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Install ##
#############

install(TARGETS pcl_scan_preprocessor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  ## Tests!
endif()
