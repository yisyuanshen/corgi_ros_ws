cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_stair LANGUAGES C CXX)

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
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs corgi_utils corgi_walk 
  sensor_msgs visualization_msgs tf2_ros tf2_eigen pcl_ros pcl_conversions pcl_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs corgi_utils corgi_walk sensor_msgs visualization_msgs tf2_ros tf2_eigen pcl_ros pcl_conversions pcl_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
  ${EIGEN3_INCLUDE_DIR}
  include
)

find_library(CORGI_UTILS corgi_utils)
find_library(CORGI_WALK corgi_walk)

add_subdirectory(src)

add_library(${PROJECT_NAME}
  src/stair_climb.cpp
  src/leg_info.cpp
  # src/dbscan.cpp
  src/plane_segmentation.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${CORGI_UTILS}
  ${CORGI_WALK}
  ${catkin_LIBRARIES}
)