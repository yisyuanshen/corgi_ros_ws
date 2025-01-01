add_library(IF InformationFilter.cpp)
target_link_libraries(IF Eigen3::Eigen Kinematic)
target_include_directories(IF PUBLIC "${PROJECT_SOURCE_DIR}/include")