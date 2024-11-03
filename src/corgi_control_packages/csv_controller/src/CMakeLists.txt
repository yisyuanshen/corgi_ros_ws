# Define the executable and link required libraries
add_executable(csv_controller ./csv_controller.cpp)
target_link_libraries(csv_controller ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
