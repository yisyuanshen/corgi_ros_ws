# Define the executable and link required libraries
add_executable(corgi_rt_control ./corgi_rt_control.cpp)
target_link_libraries(corgi_rt_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS})

add_executable(set_zero ./set_zero.cpp)
target_link_libraries(set_zero ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS})
