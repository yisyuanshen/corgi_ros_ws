#ifndef STAIRCLIMB_HPP
#define STAIRCLIMB_HPP

#include <array>
#include <vector>
#include <complex>

#include "bezier.hpp"
#include "leg_model.hpp"
#include "leg_info.hpp"
#include "trajectory_plan.hpp"

struct StairEdge {
    std::array<double, 2> edge{};
    int count = 0;
};

class StairClimb {
    public:
        // Constructor
        StairClimb(bool sim=true, std::array<double, 2> CoM_bias={0.0, 0.0}, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2);

        void initialize(double init_eta[8], double init_vel=0.0, double CoM_x=0.0);
        std::array<std::array<double, 4>, 2> step();
        void add_stair_edge(double x, double y);
        void add_stair_edge_CoM(double x, double y);
        double add_stair_edge_CoMx(double x, double y);
        void modify_last_edge_H(double y);
        void delete_last_edge();
        double get_pitch();
        std::array<double, 2> get_CoM();
        bool if_any_stair();
        bool any_no_stair();
        std::array<bool, 4> get_contact_edge_leg();
        double get_optimal_foothold(double H, bool get_front=true);
        
    private:
        /* Private function */
        void init_move_CoM_stable(int swing_leg);
        bool move_CoM_stable();
        bool slow_to_stop();
        void init_swing_same_step(int swing_leg, double front_height, double hind_height);
        bool swing_same_step();
        void init_swing_next_step(int swing_leg, double front_height, double hind_height); 
        bool swing_next_step();
        std::array<double, 2> move_consider_edge(int leg_ID, std::array<double, 2> move_vec);
        std::array<double, 2> move_edge(int leg_ID, std::array<double, 2> contact_p, double contact_alpha, double tol = 1e-4, size_t max_iter = 10);
        std::array<double, 2> objective_edge(const std::array<double, 2>& d_q, const std::array<double, 2>& current_q, std::array<double, 2> contact_p, double contact_alpha);
        bool determine_next_foothold();
        std::array<double, 2> get_foothold(double theta, double beta, bool contact_upper=true, bool contact_lower=true);
        void update_hip();

        /* Class*/
        LegModel leg_model;
        LegInfo leg_info[4] = {LegInfo(0), LegInfo(1), LegInfo(2), LegInfo(3)};
        LinearParaBlend para_traj[2];

        /* Constant value */
        const std::array<std::array<int, 2>, 4> other_side_leg = {{{3, 1},    // front-hind, left-right
                                                             {2, 0}, 
                                                             {1, 3}, 
                                                             {0, 2}}};
        const double BL;  // body length
        const double BW;  // body width
        const double BH;  // body height
        const std::array<double, 2> CoM_bias;
        const double swing_time = 0.2;                                                
        const double max_velocity = 0.1; // m/s, max velocity of CoM
        const std::array<double, 2> acc = {max_velocity / 0.3, max_velocity / 1.5}; // m/s^2, acceleration of CoM
        const double stability_margin = 0.03;
        const std::array<int, 4> swing_sequence = {0, 2, 1, 3}; // sequence of swing leg 
        const double keep_edge_d = 0.03;
        const double stand_height_on_stair_front = 0.30;
        const double stand_height_on_stair_hind  = 0.30;
        const double step_length_up_stair = 0.3;
        const double min_swing_time_cw   = 2.5, 
                     min_swing_time_ccw  = 1.0, 
                     min_swing_time_step = 1.0;
        const double min_foothold_distance = 0.1;
        const std::vector<std::pair<double, double>> foothold_table_hind = {
            {0.060, 0.07010}, {0.065, 0.11260}, {0.070, 0.12560},
            {0.075, 0.13540}, {0.080, 0.14340}, {0.085, 0.15020},
            {0.090, 0.15610}, {0.095, 0.16130}, {0.100, 0.16600},
            {0.105, 0.17010}, {0.110, 0.17390}, {0.115, 0.17700},
            {0.120, 0.18020}, {0.125, 0.18190}, {0.130, 0.18380},
            {0.135, 0.18510}, {0.140, 0.18600}, {0.145, 0.18650},
            {0.150, 0.18680}, {0.155, 0.18640}, {0.160, 0.18560},
            {0.165, 0.18460}, {0.170, 0.18320}, {0.175, 0.18120},
            {0.180, 0.17880}, {0.185, 0.17620}, {0.190, 0.16150}
        };
        const std::vector<std::pair<double, double>> foothold_table_front = {
            {0.060, 0.07010}, {0.065, 0.06720}, {0.070, 0.06360},
            {0.075, 0.05950}, {0.080, 0.05490}, {0.085, 0.05120},
            {0.090, 0.05520}, {0.095, 0.06500}, {0.100, 0.06960},
            {0.105, 0.07580}, {0.110, 0.08380}, {0.115, 0.08540},
            {0.120, 0.08970}, {0.125, 0.09420}, {0.130, 0.09700},
            {0.135, 0.10330}, {0.140, 0.10350}, {0.145, 0.10530},
            {0.150, 0.10750}, {0.155, 0.10940}, {0.160, 0.11170},
            {0.165, 0.11380}, {0.170, 0.11560}, {0.175, 0.11530},
            {0.180, 0.11670}, {0.185, 0.11720}, {0.190, 0.11690}
        };
        
        /* Variable */
        int rate;
        double dS;
        double incre_duty;
        std::array<double, 2> velocity = {0.0, 0.0};    // v_x, v_y of CoM 
        double stand_height = 0.25;
        double step_length  = 0.3;
        double step_height  = 0.04; // step height for swing on same step
        double step_height_up  = 0.08; // step height for swing when hip move up
        std::array<SwingProfile, 4> sp;
        int swing_count;
        double vel_incre;   // velocity increment for x velocity of CoM

        // State
        std::array<double, 4> theta;
        std::array<double, 4> beta;
        std::array<double, 2> CoM;
        std::array<bool, 4> enter_wheel_mode;
        std::array<bool, 4> wheel_mode;
        std::array<std::array<double, 2>, 4> hip;
        std::array<std::array<double, 2>, 4> last_hip;
        double pitch;
        int swing_leg;

        std::vector<StairEdge> stair_edge[4];
        StairEdge last_stair_edge[4];
        int stair_count;

        enum STATES {MOVE_STABLE, SLOW_DOWN, SWING_SAME, SWING_NEXT, END};
        STATES state;
        STATES last_state;

        // Intermediate variables
        std::array<double, 2> result_eta;
        std::string touch_rim_list[3] = {"G", "L_l", "L_r"};
        std::vector<std::vector<int>> touch_rim_idx = {{3}, {1, 2}, {4, 5}};

        /* Variable for move_CoM_stable */
        std::array<double, 2> CoM_offset;
        int move_dir;
        /* Variable for swing_same_step */
        double final_CoM_height;
        double front_height, hind_height;
        double t_f_x, t_f_y, t_f;
        double coeff_a, coeff_b;
        double local_max_velocity;
        double margin_d;
        int total_steps, step_count;
        /* Variable for swing_next_step */
        std::array<double, 4> last_theta, last_beta;
        std::array<std::array<double, 2>, 4> last2_hip;
        bool is_clockwise;
        double coeff_a_x, coeff_b_x, coeff_a_y, coeff_b_y;
        double first_ratio, second_ratio;
        bool first_in, second_in, third_in;
        std::array<double, 2> init_CoM, final_CoM, final_hip;
        double final_theta, final_beta, init_pitch, final_pitch;
        double init_front_height, init_hind_height;
};//end class StairClimb

#endif // STAIRCLIMB_HPP
