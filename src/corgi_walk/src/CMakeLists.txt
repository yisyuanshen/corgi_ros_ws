add_executable(walk_main walk_main.cpp)
target_link_libraries(walk_main ${catkin_LIBRARIES} ${CORGI_UTILS} corgi_walk)

add_executable(walk_sim walk_sim.cpp)
target_link_libraries(walk_sim ${catkin_LIBRARIES} ${CORGI_UTILS} corgi_walk)