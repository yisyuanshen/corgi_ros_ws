# Define the executable and link required libraries
add_executable(corgi_contact_estimation ./corgi_contact_estimation.cpp)
target_link_libraries(corgi_contact_estimation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
