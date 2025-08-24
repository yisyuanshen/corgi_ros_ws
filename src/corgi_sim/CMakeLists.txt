cmake_minimum_required(VERSION 3.8)

project(corgi_sim LANGUAGES C CXX)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)

add_message_files(
  FILES
  Float64Stamped.msg
  ContactPoint.msg
)

add_service_files(
  FILES
  field_get_string.srv
  get_uint64.srv
  motor_set_control_pid.srv
  node_enable_contact_points_tracking.srv
  node_get_contact_points.srv
  node_get_def.srv
  node_get_field.srv
  node_get_orientation.srv
  node_get_position.srv
  supervisor_get_from_id.srv
  set_float.srv
  set_int.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS rospy roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
  include
)

add_subdirectory(src)

catkin_install_python(PROGRAMS
  scripts/webots_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
