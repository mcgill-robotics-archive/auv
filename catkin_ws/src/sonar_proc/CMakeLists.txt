cmake_minimum_required(VERSION 2.8.3)
project(sonar_proc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
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
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(sonar_node
  src/main.cpp
  src/scan_preprocessor.cpp
)

add_dependencies(sonar_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(sonar_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS sonar_node
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
