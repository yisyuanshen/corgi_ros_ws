add_executable(stair_main_indoor stair_main_indoor.cpp)
target_link_libraries(stair_main_indoor ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(stair_main_indoor_open stair_main_indoor_open.cpp)
target_link_libraries(stair_main_indoor_open ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(stair_main_outdoor stair_main_outdoor.cpp)
target_link_libraries(stair_main_outdoor ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(stair_main_outdoor_open stair_main_outdoor_open.cpp)
target_link_libraries(stair_main_outdoor_open ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(stair_main_outdoor2 stair_main_outdoor2.cpp)
target_link_libraries(stair_main_outdoor2 ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(stair_main_outdoor3 stair_main_outdoor3.cpp)
target_link_libraries(stair_main_outdoor3 ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

# add_executable(normal_vector_plane_segmentation normal_vector_plane_segmentation.cpp)
# target_link_libraries(normal_vector_plane_segmentation ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

# add_executable(stair_edge_plane_segmentation stair_edge_plane_segmentation.cpp)
# target_link_libraries(stair_edge_plane_segmentation ${catkin_LIBRARIES})

add_executable(stair_estimator stair_estimator.cpp)
target_link_libraries(stair_estimator ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(save_motor_cmd save_motor_cmd.cpp)
target_link_libraries(save_motor_cmd ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(play_motor_cmd play_motor_cmd.cpp)
target_link_libraries(play_motor_cmd ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)