# Define the executable and link required libraries
add_executable(imp_test ./imp_test.cpp)
target_link_libraries(imp_test ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(force_estimate ./force_estimate.cpp)
target_link_libraries(force_estimate ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
