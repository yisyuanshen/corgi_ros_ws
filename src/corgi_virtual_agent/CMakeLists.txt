cmake_minimum_required(VERSION 3.8)

project(corgi_virtual_agent LANGUAGES C CXX)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(LOCAL_PACKAGE_PATH "${HOME}/.grpc_local" CACHE STRING "Path to grpc and core installation")
set(CORE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})
set(CMAKE_PREFIX_PATH "/opt/ros/noetic;${LOCAL_PACKAGE_PATH}")
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosgraph_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs rosgraph_msgs geometry_msgs)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

add_subdirectory(src)
