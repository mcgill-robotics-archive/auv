include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(buoy_broadcaster buoy_broadcaster.cpp)
target_link_libraries(buoy_broadcaster ${catkin_LIBRARIES})
