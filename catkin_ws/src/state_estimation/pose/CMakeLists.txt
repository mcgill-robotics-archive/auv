include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(XimuPublisher
    src/XimuPublisher.cpp
    src/XimuReceiver.cpp
)

add_executable(double_int_pose_test
    test/test_double_integration.cpp
)

add_executable(double_int_pose
    src/doubleIntPose.cpp
)

target_link_libraries(double_int_pose_test
    ${catkin_LIBRARIES}
)

target_link_libraries(double_int_pose
    ${catkin_LIBRARIES}
)

target_link_libraries(XimuPublisher
    ${catkin_LIBRARIES}
)
