#ifndef TRAJECTORYPLAN_HPP
#define TRAJECTORYPLAN_HPP

#include <vector>
#include <array>
#include <iostream>

// Linear function with parabolic blends
class LinearParaBlend {
    public:
        // Constructor: get parameters of linear function with parabolic blends.
        // p: via point, t: time at each p (0.0~1.0), tp: acceleration time, vi: initial velocity, vf: final velocity
        LinearParaBlend() {};  // Default constructor
        LinearParaBlend(std::vector<double> p, std::vector<double> t, double tp = 0.2, bool use_vi = true, double vi = 0, bool use_vf = true, double vf = 0);
       
        // Get point at the given time (0.0~1.0) of the function.
        double get_point(double t);

    private:
        std::vector<std::array<double, 3>> function_coeff;

        double tp;
        double p0;  // p[0] before offset 0.5 tp
        std::vector<double> time;
        std::vector<double> point;
        std::vector<double> vel;
        std::vector<double> acc;
        std::vector<double> time_interval;

};//end class LinearParaBlend

#endif // TRAJECTORYPLAN_HPP