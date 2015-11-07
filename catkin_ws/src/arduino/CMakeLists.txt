cmake_minimum_required(VERSION 2.8.3)
project(arduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_client
  rosserial_arduino
)

catkin_package(
  DEPENDS auv_msgs
)

set(BACKUP_PROJECT_BINARY_DIR ${PROJECT_BINARY_DIR})
set(PROJECT_WORKSPACE_DIR "${PROJECT_BINARY_DIR}/workspace")
set(PROJECT_LIBRARY_DIR "${PROJECT_WORKSPACE_DIR}/libraries")
# We want the files from rosserial generate to go in a slightly different folder.
set(PROJECT_BINARY_DIR ${PROJECT_LIBRARY_DIR})
rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)
set(PROJECT_BINARY_DIR ${BACKUP_PROJECT_BINARY_DIR})

# The ros_lib has to be deleted before generating a new one.
add_custom_target(
  delete_${PROJECT_NAME}_ros_lib
  COMMAND rm -rf ${PROJECT_LIBRARY_DIR}
)

add_custom_target(
  link_ino_files
  COMMAND mkdir -p ${PROJECT_WORKSPACE_DIR}
  COMMAND ln -sf ${PROJECT_SOURCE_DIR}/blinky ${PROJECT_WORKSPACE_DIR}
  COMMAND ln -sf ${PROJECT_SOURCE_DIR}/radio_interrupt ${PROJECT_WORKSPACE_DIR}
  COMMAND ln -sf ${PROJECT_SOURCE_DIR}/teensy ${PROJECT_WORKSPACE_DIR}
  COMMAND ln -sf ${PROJECT_SOURCE_DIR}/teensy_depth ${PROJECT_WORKSPACE_DIR}
)

add_dependencies(
  ${PROJECT_NAME}_ros_lib
  delete_${PROJECT_NAME}_ros_lib 
  link_ino_files
  auv_msgs_genpy
)

