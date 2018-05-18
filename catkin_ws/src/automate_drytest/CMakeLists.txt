cmake_minimum_required(VERSION 2.8.3)
project(automate_drytest)

find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  auv_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
    scripts/sensors_dry_test.py
    scripts/full_dry_test.py
    scripts/drytest_thruster.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
