add_executable(change_leg_length_same_height change_leg_length_same_height.cpp)
target_link_libraries(change_leg_length_same_height ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(change_front_legs_length change_front_legs_length.cpp)
target_link_libraries(change_front_legs_length ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(change_rear_legs_length change_rear_legs_length.cpp)
target_link_libraries(change_rear_legs_length ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)