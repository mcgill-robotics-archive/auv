add_executable(lane_detector lane_detector.cpp)
target_link_libraries(lane_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(torpedo_target torpedo_targeter.cpp)
target_link_libraries(torpedo_target ${catkin_LIBRARIES})

add_executable(bin_detector bin_detector.cpp)
target_link_libraries(bin_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
