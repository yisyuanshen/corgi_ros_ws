#include <vector>
#include <array>
#include <iostream>
#include "trajectory_plan.hpp"

LinearParaBlend::LinearParaBlend(std::vector<double> p, std::vector<double> t, double tp, bool use_vi, double vi, bool use_vf, double vf) :
    /* Initializer List */
    tp(tp)
{
    int n_points = p.size();
    if (n_points != t.size()) {
        throw std::runtime_error("p & t need to have the same size.");
    }//end if 
    if (n_points < 2) {
        throw std::runtime_error("Need at least 2 point.");
    }//end if 

    // calculate vi & vf or offset on p & t
    p0 = p[0];
    if (!use_vi) {
        vi = (p[1] - p[0]) / (t[1] - t[0]);
    } else {
        p[0] += vi * tp / 2.0;
        t[0] += 0.5 * tp;
    }//end if else
    if (!use_vf) {
        vf = (p[n_points - 1] - p[n_points - 2]) / (t[n_points - 1] - t[n_points - 2]);
    } else {
        p[n_points - 1] -= vf * tp / 2.0;
        t[n_points - 1] -= 0.5 * tp;
    }//end if else
    
    // calculate velocity of each segment
    vel.resize(n_points + 1);
    vel[0] = vi;
    for (int i=1; i<n_points; i++) {
        vel[i] = (p[i] - p[i - 1]) / (t[i] - t[i - 1]);
    }//end for
    vel[n_points] = vf;
    // calculate acceleration of each segment
    acc.resize(n_points);
    for (int i=0; i<n_points; i++) {
        acc[i] = (vel[i + 1] - vel[i]) / tp;
    }//end for
    
    // get the coefficient of the function
    function_coeff.resize(2*n_points - 1);
    function_coeff[0] = {0.5*acc[0], vel[0], p0};
    for (int i=0; i<n_points-1; i++) {
        function_coeff[2*i + 1] = {0, vel[i + 1], -t[i]*vel[i + 1] + p[i]};    // linear: constant speed
        double tmp = t[i + 1] - 0.5 * tp;    // acceleration start time
        function_coeff[2*i + 2] = {function_coeff[2*i+1][0] + 0.5*acc[i+1], function_coeff[2*i+1][1] -acc[i+1]*tmp, function_coeff[2*i+1][2] + 0.5*acc[i+1]*tmp*tmp};
    }//end for

    // Store p & t
    time = t;
    point = p;
    // create time_interval separate each segment
    time_interval.resize(2*n_points - 1);
    time_interval[0] = time[0] + 0.5 * tp;
    for (int i=0; i<n_points-1; i++) {
        time_interval[2*i+1] = time[i+1] - 0.5 * tp;
        time_interval[2*i+2] = time[i+1] + 0.5 * tp;
    }//end for
}//end LinearParaBlend

double LinearParaBlend::get_point(double t) {
    if (t < 0.0) {
        return function_coeff[0][0]*t*t + function_coeff[0][1]*t + function_coeff[0][2];
    } else if (t >= 1.0) {
        return function_coeff.back()[0]*t*t + function_coeff.back()[1]*t + function_coeff.back()[2];
    } else {
        for (size_t idx=0; idx<time_interval.size(); idx++) {
            if (t < time_interval[idx]) {
                return function_coeff[idx][0]*t*t + function_coeff[idx][1]*t + function_coeff[idx][2];
            }//end if
        }//end for
    }//end if else

    throw std::runtime_error("\"get_point\" should not reach here.");
    return 0;
}//end get_point
