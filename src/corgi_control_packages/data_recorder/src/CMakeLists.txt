# Define the executable and link required libraries
add_executable(data_recorder ./data_recorder.cpp)
target_link_libraries(data_recorder ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
