cmake_minimum_required(VERSION 2.8.3)
project(auv_base)

find_package(catkin REQUIRED COMPONENTS
  pointgrey_camera_driver
  nodelet
)

catkin_package()
