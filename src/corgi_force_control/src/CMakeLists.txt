add_executable(imp_cmd_pub imp_cmd_pub.cpp)
target_link_libraries(imp_cmd_pub ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_UTILS})

add_executable(admittance_control admittance_control.cpp)
target_link_libraries(admittance_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(force_control force_control.cpp)
target_link_libraries(force_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(exp_h15 exp_h15.cpp)
target_link_libraries(exp_h15 ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(exp_h18 exp_h18.cpp)
target_link_libraries(exp_h18 ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(exp_h21 exp_h21.cpp)
target_link_libraries(exp_h21 ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(exp_h24 exp_h24.cpp)
target_link_libraries(exp_h24 ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(exp_sim_stay exp_sim_stay.cpp)
target_link_libraries(exp_sim_stay ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(exp_height exp_height.cpp)
target_link_libraries(exp_height ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)