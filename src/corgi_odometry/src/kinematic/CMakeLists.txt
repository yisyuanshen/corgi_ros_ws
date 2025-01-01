add_library(Kinematic LinkLegModel.cpp Leg.cpp)
target_link_libraries(Kinematic Eigen3::Eigen)
target_include_directories(Kinematic PUBLIC "${PROJECT_SOURCE_DIR}/include")

