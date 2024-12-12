# Define the executable and link required libraries
add_executable(corgi_sim ./corgi_sim.cpp)
target_link_libraries(corgi_sim ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})