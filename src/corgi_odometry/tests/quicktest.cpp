/**
 * @file quicktest.cpp
 * 
 * @author peichunhuang
 */
#include "kinematic/Leg.hpp"

int main() {
    Leg leg(Eigen::Vector3f(0, 0, 0));
    leg.Calculate(160.0 / 180.0 * M_PI_F, 0, 0, 0, 0, 0);
    std::cout << std::abs(leg.G) << "\n";
}