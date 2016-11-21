include_directories(. ${catkin_INCLUDE_DIRS})
add_executable(lane_detector laneDetector.cpp)
target_link_libraries(lane_detector ${catkin_LIBRARIES})
