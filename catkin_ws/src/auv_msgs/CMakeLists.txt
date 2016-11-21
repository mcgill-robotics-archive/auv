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
  SetPosition.msg
  SetVelocity.msg
  MotorCommands.msg
  SolenoidCommands.msg
  VisualServoCommand.msg

  # Added by State Estimation
  RangeBearingElevation.msg
  SlamEstimate.msg
  Lane.msg

  # Added by CV
  CVTarget.msg

  # Added by IO
  RadioCommands.msg
  CPUTemperature.msg

  # Added by hydrophones
  Signals.msg

  # Added by Taskr
  TaskStatus.msg
)

add_service_files(FILES
  ConfidenceRadius.srv
  XYConfidenceRadius.srv
  Covariance.srv
)

add_action_files(
  DIRECTORY action
  FILES SetVelocity.action
  SetPosition.action
  SetCVTarget.action
  InitializeHorizon.action
  VisualServo.action
)

generate_messages(DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs)
