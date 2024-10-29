cmake_minimum_required(VERSION 3.0.2)
project(corgi_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Headers.msg
  MotorCmd.msg
  MotorCmdStamped.msg
  MotorState.msg
  MotorStateStamped.msg
  PowerCmdStamped.msg
  PowerStateStamped.msg
  TriggerStamped.msg
  SensorEnableStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
