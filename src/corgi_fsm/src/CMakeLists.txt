add_executable(fsm_control fsm_control.cpp)
target_link_libraries(fsm_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${WHEEL_CONTROL} ${CORGI_TRANSFORM} Eigen3::Eigen)

add_executable(fsm_cmd_pub fsm_cmd_pub.cpp)
target_link_libraries(fsm_cmd_pub ${catkin_LIBRARIES} ${CMAKE_DL_LIBS}  ${WHEEL_CONTROL} ${CORGI_UTILS} Eigen3::Eigen)