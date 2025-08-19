cmake_minimum_required(VERSION 3.0.2)
project(corgi_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Headers.msg
  ImpedanceCmd.msg
  ImpedanceCmdStamped.msg
  ForceState.msg
  ForceStateStamped.msg
  MotorCmd.msg
  MotorCmdStamped.msg
  MotorState.msg
  MotorStateStamped.msg
  PowerCmdStamped.msg
  PowerStateStamped.msg
  TriggerStamped.msg
  SensorEnableStamped.msg
  SteeringCmdStamped.msg
  SteeringStateStamped.msg
  WheelCmd.msg
  SimDataStamped.msg
  IMUStamped.msg
  FsmCmdStamped.msg
  FsmStateStamped.msg
  ContactState.msg
  ContactStateStamped.msg
  StairPlanes.msg
)

add_service_files(
  FILES
  imu.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation std_msgs geometry_msgs sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
