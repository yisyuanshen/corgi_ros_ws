# Define the executable and link required libraries
add_executable(imp_test ./imp_test.cpp)
target_link_libraries(imp_test ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} Eigen3::Eigen)

add_executable(force_estimation ./force_estimation.cpp)
target_link_libraries(force_estimation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen)

add_executable(motor_state_pub ./motor_state_pub.cpp)
target_link_libraries(motor_state_pub ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})