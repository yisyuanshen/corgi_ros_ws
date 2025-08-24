cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_terrain_estimator LANGUAGES C CXX)
set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)
set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")
set(EIGEN3_INCLUDE_DIR "${LOCAL_PACKAGE_PATH}/include/eigen3")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosgraph_msgs geometry_msgs sensor_msgs corgi_msgs corgi_utils corgi_force_estimation)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs rosgraph_msgs geometry_msgs sensor_msgs corgi_msgs corgi_utils corgi_force_estimation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
  ${EIGEN3_INCLUDE_DIR}
  include
)

find_library(CORGI_UTILS corgi_utils)
find_library(CORGI_FORCE_EST corgi_force_estimation)

add_subdirectory(src)
