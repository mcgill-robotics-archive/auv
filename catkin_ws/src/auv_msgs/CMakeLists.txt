cmake_minimum_required(VERSION 2.8.3)
project(auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  # Added by Controls
  MotorCommands.msg
  ThrusterCommands.msg
  SolenoidCommands.msg
  PneumaticCommands.msg
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
  Covariance.srv
  XYConfidenceRadius.srv
)

add_action_files(
  DIRECTORY action
  FILES
    Hydrophones.action
    InitializeHorizon.action
    SetVelocity.action  # Deprecated
)

generate_messages(DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  sensor_msgs
  std_msgs
)
