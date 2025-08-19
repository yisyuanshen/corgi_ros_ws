cmake_minimum_required(VERSION 3.8)

project(corgi_odometry LANGUAGES C CXX)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs sensor_msgs corgi_utils)
catkin_package(CATKIN_DEPENDS roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs sensor_msgs corgi_utils)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
)

find_library(CORGI_UTILS corgi_utils)

add_subdirectory(src/kinematic)
add_subdirectory(src/KLD_estimation)
add_subdirectory(src/corgi_odometry)
add_subdirectory(tests)
