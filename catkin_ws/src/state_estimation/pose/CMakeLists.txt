include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(XimuPublisher
    src/XimuPublisher.cpp
    src/XimuReceiver.cpp
)

add_executable(doubleIntPose
    src/doubleIntPose.cpp
)

target_link_libraries(doubleIntPose
    ${catkin_LIBRARIES}
)

target_link_libraries(XimuPublisher
    ${catkin_LIBRARIES}
)
