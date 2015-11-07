add_executable(pose_ukf
  src/ros_pose.cpp
  src/ukf.cpp
  src/ukf_pose.cpp
)
target_link_libraries(pose_ukf ${catkin_LIBRARIES})

add_executable(slam_ukf
  src/ros_slam.cpp
  src/ukf.cpp
  src/ukf_slam.cpp
)
add_dependencies(slam_ukf auv_msgs_generate_messages_cpp)
target_link_libraries(slam_ukf ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
include_directories(src)
# catkin_add_gtest(${PROJECT_NAME}-test 
#   test/test_ukf.cpp
#   src/ukf.cpp
#   src/ukf_pose.cpp
#   src/ukf_slam.cpp
# )

## Simulates visual odometry and CV/sonar inputs ##
#add_executable(sim_slam_input test/sim_slam_input.cpp)
#add_dependencies(sim_slam_input auv_msgs_generate_messages_cpp)
#target_link_libraries(sim_slam_input ${catkin_LIBRARIES})

## Verifies output of slam using simulated inputs ##
add_executable(verify_slam test/verify_slam.cpp)
target_link_libraries(verify_slam ${catkin_LIBRARIES})
