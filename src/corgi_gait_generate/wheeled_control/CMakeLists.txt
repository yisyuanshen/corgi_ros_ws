cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(wheeled_control LANGUAGES C CXX)

set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)
set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    rosgraph_msgs 
    geometry_msgs 
    corgi_msgs 
    joy 
    sensor_msgs
  )


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    rosgraph_msgs 
    geometry_msgs 
    corgi_msgs 
    joy 
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
  ${catkin_LIBRARIES}
  include
)

add_subdirectory(src)

add_library(${PROJECT_NAME}
  src/joystick_control.cpp
  src/wheeled.cpp
)