# Define the executable and link required libraries
add_executable(csv_control ./csv_control.cpp)
target_link_libraries(csv_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
