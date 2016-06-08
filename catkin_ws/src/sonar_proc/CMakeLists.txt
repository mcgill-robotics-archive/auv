cmake_minimum_required(VERSION 2.8.3)
project(sonar_proc)

find_package(catkin REQUIRED COMPONENTS
             roscpp
            rospy
            std_msgs
            pcl_ros
            sensor_msgs)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    pcl_ros
    sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(sonar_proc src/main.cpp
               src/scan_preprocessor.cpp)

target_link_libraries(sonar_proc ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(sonar_proc sonar_proc_generate_messages_cpp)
