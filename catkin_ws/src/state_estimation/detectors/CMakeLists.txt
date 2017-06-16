add_executable(lane_detector laneDetector.cpp)
add_executable(torpedo_target torpedoTargeter.cpp)
target_link_libraries(lane_detector ${catkin_LIBRARIES})
target_link_libraries(torpedo_target ${catkin_LIBRARIES})

