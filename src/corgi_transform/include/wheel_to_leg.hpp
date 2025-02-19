#ifndef WHEEL_TO_LEG_HPP
#define WHEEL_TO_LEG_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_model.hpp"
#include "bezier.hpp"

class WheelToLegTransformer {
    public:
        int stage = 0;
        bool transform_finished = false;
        int stay_time_step = 0;
        double total_move_dist = 0;

        WheelToLegTransformer(bool sim=true);

        void initialize(double init_eta[8]);

        double round_3(double value);
        double round_6(double value);
        double find_closest_beta(double target_beta, double ref_beta);
        double find_smaller_closest_beta(double target_beta, double ref_beta);
        std::array<double, 2> find_hybrid_steps(double RH_beta, double LH_beta, double body_angle);
        std::array<std::array<double, 4>, 2> step();

    private:
        LegModel leg_model;
        double G_p[2] = {0, 0};

        std::array<double, 4> curr_theta;
        std::array<double, 4> curr_beta;

        const double BL = 0.444;
        const double BW = 0.4;
        const double BH = 0.2;

        double last_transform_step_x = 0.15;
        double body_vel = 0.1;
        double stance_height = 0.2;
        double step_height = 0.05;
        double dt = 0.001;

        double wheel_delta_beta;

        int step_count = 0;

        double delta_time_step;
        double body_move_dist;

        double RF_target_beta, RF_target_theta;
        double LF_target_beta, LF_target_theta;

        double RF_delta_beta, RF_delta_theta;
        double LF_delta_beta, LF_delta_theta;

        std::array<double, 2> hybrid_steps;
        int step_num;
        double step_length;

        double body_angle;

        int delta_time_step_each;
        std::array<double, 2> p_lo, p_td;
        SwingProfile sp;

        std::array<double, 2> curve_point_temp;
        double curve_point[2];
        std::array<double, 2> swing_eta;

        std::array<double, 2> stance_eta;
        std::array<double, 2> move_vector;

        int traj_idx;

        double transform_start_beta, transform_start_theta;
        double transform_target_beta, transform_target_theta;
        double transform_delta_beta, transform_delta_theta;

        double last_start_beta, last_start_theta;
        double last_target_theta, last_target_beta;
        double last_delta_beta, last_delta_theta;

        double hind_body_height;
        
        double LF_target_pos[2], RF_target_pos[2];
};




#endif
