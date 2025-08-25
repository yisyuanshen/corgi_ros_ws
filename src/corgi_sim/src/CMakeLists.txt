add_library(Contact contact_processor.cpp)
target_link_libraries(Contact Eigen3::Eigen)
target_include_directories(Contact PUBLIC "${PROJECT_SOURCE_DIR}/include")

# Define the executable and link required libraries
add_executable(corgi_sim_pos ./corgi_sim_pos.cpp)
target_link_libraries(corgi_sim_pos ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} Eigen3::Eigen)
add_dependencies(corgi_sim_pos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(corgi_sim_trq ./corgi_sim_trq.cpp)
target_link_libraries(corgi_sim_trq ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} Eigen3::Eigen Contact)
add_dependencies(corgi_sim_trq ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})