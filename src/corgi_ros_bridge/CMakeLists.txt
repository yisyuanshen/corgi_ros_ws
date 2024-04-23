cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_ros_bridge C CXX)

option(BUILD_WITH_ROS "set flag off when not build in catkin workerspace" ON)
set(LOCAL_PACKAGE_PATH "${HOME}/.grpc_local" CACHE STRING "where you install grpc and core")
set(CORE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (BUILD_WITH_ROS)
  set(CMAKE_PREFIX_PATH "/opt/ros/noetic;${LOCAL_PACKAGE_PATH}")
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation)
    add_message_files(
    FILES
    RobotStamped.msg
    motor.msg
    LegStamped.msg
    force.msg
    impedance.msg
    )

    generate_messages(DEPENDENCIES std_msgs geometry_msgs)
  catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime)
  include_directories(${catkin_INCLUDE_DIRS})
else() 
  set(CMAKE_PREFIX_PATH ${CORE_PREFIX_PATH})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)
endif()

set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fPIC")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(mip REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

# generate proto file c/c++ interface
set(PROTO_PATH "${CMAKE_CURRENT_LIST_DIR}/protos")
file(GLOB PROTO_FILES "${PROTO_PATH}/*.proto")

foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(FILE_NAME ${PROTO_FILE} NAME_WE)
    list(APPEND PROTONAMES ${FILE_NAME})
    message(${FILE_NAME})
endforeach()

foreach(PROTONAME ${PROTONAMES})
    add_custom_command(
      OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROTONAME}.pb.cc"
      "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROTONAME}.pb.h"
      COMMAND ${_PROTOBUF_PROTOC}
        --cpp_out "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
        -I "${PROTO_PATH}"
        "${PROTO_PATH}/${PROTONAME}.proto"
      DEPENDS "${PROTO_PATH}/${PROTONAME}.proto")

    add_library("${PROTONAME}_proto"
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROTONAME}.pb.cc"
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROTONAME}.pb.h")
    target_link_libraries("${PROTONAME}_proto"
      ${_REFLECTION}
      ${_GRPC_GRPCPP}
      ${_PROTOBUF_LIBPROTOBUF} ${_CORE_LIBRARIES})
endforeach()
# Include generated *.pb.h files
include_directories("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")


add_subdirectory(src)
