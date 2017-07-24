cmake_minimum_required(VERSION 2.8.3)
project(auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  FILES
  # Added by Controls
  MotorCommands.msg
  SolenoidCommands.msg
  SetVelocity.msg

  # Added by State Estimation
  RangeBearingElevation.msg
  SlamEstimate.msg
  TorpedoTarget.msg

  # Added by IO
  RadioCommands.msg
  CPUTemperature.msg

  # Added by hydrophones
  Signals.msg

  # Added by Taskr
  TaskStatus.msg

  # Added by Sonar
  TaskPointsArray.msg
)

add_service_files(FILES
  ConfidenceRadius.srv
  XYConfidenceRadius.srv
  Covariance.srv
)

add_action_files(
  DIRECTORY action
  FILES
    SetVelocity.action  # Deprecated
    InitializeHorizon.action
)

generate_messages(DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs)
