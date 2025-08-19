# Define the executable and link required libraries
add_executable(corgi_odometry ./corgi_odometry.cpp)
target_link_libraries(corgi_odometry ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} Eigen3::Eigen Kinematic IF)

add_executable(corgi_z_position ./corgi_z_position.cpp)
target_link_libraries(corgi_z_position ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen IF)