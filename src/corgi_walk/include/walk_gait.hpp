#ifndef WALKGAIT_HPP
#define WALKGAIT_HPP

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>

#include "leg_model.hpp"
#include "bezier.hpp"
class WalkGait {
    public:
        WalkGait(bool sim=true, double CoM_bias=0.0, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2);

        void initialize(double init_eta[8], double step_length_=0.3);
        std::array<std::array<double, 4>, 2> step();
        void set_velocity(double new_value);
        void set_stand_height(double new_value);
        void set_step_length(double new_value);
        void set_step_height(double new_value);
        void set_curvature(double new_value);
        void set_eta(std::array<std::array<double, 4>, 2> eta_);
        void set_duty(std::array<double, 4> duty_);
        std::array<int, 4> get_step_count();
        std::array<int, 4> get_swing_phase();
        std::array<double, 4> get_duty();
        bool if_touchdown();
        int get_touchdown_leg();

    private:
        LegModel leg_model;

        // Constant value
        const double BL;  // body length
        const double BW;  // body width
        const double BH;  // body height
        const double CoM_bias;
        const double swing_time = 0.2;

        // Variable
        int rate;
        double dS;
        double incre_duty;
        double velocity     = 0.1;
        double stand_height = 0.25;
        double step_length  = 0.3;
        double step_height  = 0.04;
        double curvature    = 0.0;  // +: turn left, -:turn right, 0: straight

        // State
        std::array<double, 4> theta;
        std::array<double, 4> beta;
        std::array<std::array<double, 2>, 4> foothold;
        std::array<std::array<double, 2>, 4> hip;
        std::array<std::array<double, 2>, 4> next_hip;

        std::array<double, 4> duty;
        std::array<int, 4> swing_phase = {0, 0, 0, 0};
        std::array<int, 4> step_count  = {0, 0, 0, 0};
        std::array<double, 4> current_step_length = {step_length, step_length, step_length, step_length};
        std::array<double, 4> next_step_length    = {step_length, step_length, step_length, step_length};
        double new_step_length = step_length;
        int direction = 1;
        int touchdown;

        // Intermediate variables
        int current_rim;
        std::string touch_rim_list[5] = {"G", "L_l", "L_r", "U_l", "U_r"};
        int touch_rim_idx[5] = {3, 2, 4, 1, 5};
        double swing_phase_ratio;
        std::array<double, 2> curve_point_temp;
        std::array<double, 2> result_eta;
        std::array<double, 2> p_lo;
        std::array<double, 2> p_td;
        std::array<SwingProfile, 4> sp;

        // For turning 
        double outer_radius;
        double inner_radius;
        double diff_step_length = 0.0;  // Differential step length 
        double new_diff_step_length = 0.0;  // New differential step length
        double diff_dS = 0.0;   // Differential dS
        int sign_diff[4];   // Differential sign
};//end class WalkGait

#endif // WALKGAIT_HPP
