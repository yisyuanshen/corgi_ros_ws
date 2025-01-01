# Define the executable and link required libraries
add_executable(corgi_odometry ./corgi_odometry.cpp)
target_link_libraries(corgi_odometry ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
