add_executable(lane_detector lane_detector.cpp)
add_executable(torpedo_target torpedo_targeter.cpp)

target_link_libraries(lane_detector ${catkin_LIBRARIES})
target_link_libraries(torpedo_target ${catkin_LIBRARIES})

