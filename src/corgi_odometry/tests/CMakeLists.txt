add_executable(quick_test quicktest.cpp)
target_link_libraries(quick_test Eigen3::Eigen Kinematic)
target_include_directories(quick_test PUBLIC "${PROJECT_SOURCE_DIR}/include")

add_executable(if_estimate IF_main.cpp)
target_link_libraries(if_estimate Eigen3::Eigen ${_CORE_LIBRARIES} Kinematic IF pthread)
target_include_directories(if_estimate PUBLIC "${PROJECT_SOURCE_DIR}/include")