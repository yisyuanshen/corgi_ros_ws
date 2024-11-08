cmake_minimum_required(VERSION 3.8)

project(corgi_panel)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rosgraph_msgs
  geometry_msgs
  corgi_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    rosgraph_msgs
    geometry_msgs
    corgi_msgs
)

catkin_install_python(PROGRAMS
  scripts/corgi_control_panel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
