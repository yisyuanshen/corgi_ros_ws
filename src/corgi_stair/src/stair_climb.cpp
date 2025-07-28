#include <iostream>
#include <cmath>
#include <stdexcept>
#include <array>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include "bezier.hpp"
#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "trajectory_plan.hpp"
#include "stair_climb.hpp"

// #define BEZIER_CURVE_SWING 1
#define CHANGE_FIRST_SWING_LEG 1
#define WHEEL_MODE_WHEN_CONTACT 1

StairClimb::StairClimb(bool sim, std::array<double, 2> CoM_bias, int rate, double BL, double BW, double BH) : 
    /* Initializer List */
    leg_model(sim),
    CoM_bias(CoM_bias),
    rate(rate), 
    BL(BL),
    BW(BW),
    BH(BH)
{
    // Initialize
    state = MOVE_STABLE;
    last_state = MOVE_STABLE;
    vel_incre = acc[0] / rate;
    stair_count = 0;
}//end StairClimb

void StairClimb::initialize(double init_eta[8], double init_vel, double CoM_x) {
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4]  = {-init_eta[1], init_eta[3], init_eta[5], -init_eta[7]};
    velocity[0] = init_vel; // x velocity of CoM
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    int current_rim = 0;
    for (int i=0; i<4; i++) {
        leg_model.contact_map(init_theta[i], init_beta[i]);
        current_rim = leg_model.rim;
        leg_model.forward(init_theta[i], init_beta[i]);
        if (current_rim == 1) {
            relative_foothold[i][0] = leg_model.U_l[0];
        } else if (current_rim == 2) {
            relative_foothold[i][0] = leg_model.L_l[0];
        } else if (current_rim == 3) {
            relative_foothold[i][0] = leg_model.G[0];
        } else if (current_rim == 4) {
            relative_foothold[i][0] = leg_model.L_r[0];
        } else if (current_rim == 5) {
            relative_foothold[i][0] = leg_model.U_r[0];
        } else {
            std::cout << "Leg cannot contact ground if use the given initial theta/beta." << std::endl;
        }//end if else
        relative_foothold[i][1] = -stand_height;
    }//end for
    // Get first swing leg  
    int first_swing_leg = 0;
    for (int i=1; i<4; i++) {
        if (relative_foothold[i][0] < relative_foothold[first_swing_leg][0]) {
            first_swing_leg = i;
        }//end if
    }//end for 
    switch (first_swing_leg) {
        case 0: swing_count = 0; break;    
        case 1: swing_count = 2; break;
        case 2: swing_count = 1; break;
        case 3: swing_count = 3; break;
        default: std::cout << "Error in determining first swing leg." << std::endl; break;
    }//end switch
    // Get foothold in world coordinate
    CoM = {CoM_x, stand_height};
    pitch = 0;
    hip = {{{CoM_x+BL/2, stand_height} ,
            {CoM_x+BL/2, stand_height} ,
            {CoM_x-BL/2, stand_height},
            {CoM_x-BL/2, stand_height}}};
    front_height = stand_height;
    hind_height  = stand_height;
    // Initial theta/beta
    for (int i=0; i<4; i++) {
        theta[i] = init_theta[i];
        beta[i]  = init_beta[i];
        enter_wheel_mode[i] = false;
        wheel_mode[i] = false;
        last_stair_edge[i].edge[0] = -100;
    }//end for
    init_move_CoM_stable(swing_sequence[swing_count % 4]);
}//end initialize

std::array<std::array<double, 4>, 2> StairClimb::step() {
    // state machine
    bool finish_move = false;
    switch (this->state) {
        case MOVE_STABLE:
            if (last_state != state) {
                init_move_CoM_stable(swing_sequence[swing_count % 4]);
            }//end if
            finish_move = move_CoM_stable();
            break;
        case SLOW_DOWN:
            finish_move = slow_to_stop();
            break;
        case SWING_SAME:
            if (last_state != state) {
                swing_leg = swing_sequence[swing_count % 4];
                if (!stair_edge[swing_leg].empty()) {
                    if (leg_info[swing_leg].next_up) {
                    // if (true) {
                        if (swing_leg == 0 || swing_leg == 1) {
                            front_height = leg_info[swing_leg].foothold[1] + stand_height_on_stair_front;
                        } else {
                            hind_height  = leg_info[swing_leg].foothold[1] + stand_height_on_stair_hind;
                        }//end if else
                    }//end if
                }//end if
                init_swing_same_step(swing_sequence[swing_count % 4], front_height, hind_height);
            }//end if
            finish_move = swing_same_step();
            break;
        case SWING_NEXT:
            if (last_state != state) {
                swing_leg = swing_sequence[swing_count % 4];
                this->is_clockwise = true;
                int other_leg = other_side_leg[swing_leg][1];
                if (swing_leg == 0 || swing_leg == 1) {
                    if (!stair_edge[0].empty() && !stair_edge[1].empty()) { // at most only one will be empty (edge of swing leg must not be empty)
                        if (stair_edge[0].front().count != stair_edge[1].front().count) {   // second swing leg
                            // double stand_height_on_stair = stair_edge[swing_leg].size() >= 2? stand_height_on_stair_front : stand_height;
                            // double stand_height_on_stair = leg_info[swing_leg].next_up? stand_height_on_stair_front : stand_height;
                            double D = stair_edge[other_leg].front().edge[0] - last_stair_edge[other_leg].edge[0];
                            double stand_height_on_stair = D<0.3? stand_height_on_stair_front : stand_height;
                            front_height = stair_edge[swing_leg].front().edge[1] + stand_height_on_stair;
                            this->is_clockwise = false;
                        } else {    // first swing leg
                            // front_height = stair_edge[swing_leg].front().edge[1] + stand_height;
                        }//end if else
                    } else {    // second swing leg, first swing leg is on the upper ground
                        this->is_clockwise = false;
                        // double stand_height_on_stair = stair_edge[swing_leg].size() >= 2? stand_height_on_stair_front : stand_height;
                        double stand_height_on_stair = leg_info[swing_leg].next_up? stand_height_on_stair_front : stand_height;
                        front_height = stair_edge[swing_leg].front().edge[1] + stand_height_on_stair;
                    }//end if else
                } else {
                    if (!stair_edge[2].empty() && !stair_edge[3].empty()) { // at most only one will be empty (edge of swing leg must not be empty)
                        if (stair_edge[2].front().count != stair_edge[3].front().count) {   // second swing leg
                            // double stand_height_on_stair = stair_edge[swing_leg].size() >= 2? stand_height_on_stair_hind : stand_height;
                            // double stand_height_on_stair = leg_info[swing_leg].next_up? stand_height_on_stair_hind : stand_height;
                            double D = stair_edge[other_leg].front().edge[0] - last_stair_edge[other_leg].edge[0];
                            double stand_height_on_stair = D<0.3? stand_height_on_stair_hind : stand_height;
                            hind_height = stair_edge[swing_leg].front().edge[1] + stand_height_on_stair;
                            this->is_clockwise = false;
                        } else {    // first swing leg
                            // hind_height = stair_edge[swing_leg].front().edge[1] + stand_height;
                        }//end if else
                    } else {    // second swing leg, first swing leg is on the upper ground
                        this->is_clockwise = false;
                        // double stand_height_on_stair = stair_edge[swing_leg].size() >= 2? stand_height_on_stair_hind : stand_height;
                        double stand_height_on_stair = leg_info[swing_leg].next_up? stand_height_on_stair_hind : stand_height;
                        hind_height = stair_edge[swing_leg].front().edge[1] + stand_height_on_stair;
                    }//end if else  
                }//end if else
                init_swing_next_step(swing_sequence[swing_count % 4], front_height, hind_height);
            }//end if
            finish_move = swing_next_step();
            break;
        default:
            break;
    }//end switch
    // if (last_state != state && (state == SWING_SAME || state == SWING_NEXT)) {
        // std::cout << "State " << this->state << std::endl;
        // std::cout << "Leg " << swing_leg << " next_foothold: " << leg_info[swing_leg].next_foothold[0] << ", " << leg_info[swing_leg].next_foothold[1] << std::endl;
        // std::cout << "front_height:" << this->front_height << std::endl;
        // std::cout << "hind_height:" << this->hind_height << std::endl;
    // }
    last_state = state;

    // next state
    switch (this->state) {
        case MOVE_STABLE:
            if (finish_move) {
                // if (velocity[0] >= 0.0) {
                if (false) {
                    bool up_stair = determine_next_foothold();
                    state = up_stair? SWING_NEXT : SWING_SAME;
                    if (!this->if_any_stair()) {
                        state = END;
                    }//end if
                } else {
                    state = SLOW_DOWN;
                }//end if else
            }//end if
            break;
        case SLOW_DOWN:
            if (finish_move) {
                if (move_dir * (CoM[0] + CoM_offset[0]) > move_dir * ((leg_info[(swing_leg+1)%4].foothold[0] + leg_info[(swing_leg+3)%4].foothold[0]) / 2) + stability_margin) {
                    bool up_stair = determine_next_foothold();
                    state = up_stair? SWING_NEXT : SWING_SAME;
                } else {
                    state = MOVE_STABLE;
                }

                if (!this->if_any_stair()) {
                    state = END;
                }//end if
            }//end if
            break;
        case SWING_SAME:
            if (finish_move) {
                state = MOVE_STABLE;
                leg_info[swing_leg].foothold = leg_info[swing_leg].next_foothold;
                leg_info[swing_leg].contact_edge = false;
                enter_wheel_mode[swing_leg] = false; // exit wheel mode
                wheel_mode[swing_leg] = false;
                swing_count ++;
            }//end if
            break;
        case SWING_NEXT:
            if (finish_move) {
                state = MOVE_STABLE;
                leg_info[swing_leg].stair_count = stair_edge[swing_leg].front().count;
                leg_info[swing_leg].foothold = leg_info[swing_leg].next_foothold;
                leg_info[swing_leg].contact_edge = false;
                enter_wheel_mode[swing_leg] = false; // exit wheel mode
                wheel_mode[swing_leg] = false;
                last_stair_edge[swing_leg] = stair_edge[swing_leg].front();
                stair_edge[swing_leg].erase(stair_edge[swing_leg].begin());
                swing_count ++;
            }//end if
            break;
        case END:
            std::cout << "End of stair climbing." << std::endl;
            break;
        default:
            break;
    }//end switch

    return {theta, beta};
}//end step

void StairClimb::add_stair_edge(double x, double y) {
    stair_count ++;
    std::cout << "Add stair edge at (" << x << ", " << y << ") relative to origin." << std::endl;
    stair_edge[0].push_back({{x, y}, stair_count});
    stair_edge[1].push_back({{x, y}, stair_count});
    stair_edge[2].push_back({{x, y}, stair_count});
    stair_edge[3].push_back({{x, y}, stair_count});
}//end add_stair_edge

void StairClimb::add_stair_edge_CoM(double x, double y) {
    stair_count ++;
    std::cout << "Add stair edge at (" << x << ", " << y << ") relative to CoM." << std::endl;
    x += CoM[0];
    y += CoM[1];
    stair_edge[0].push_back({{x, y}, stair_count});
    stair_edge[1].push_back({{x, y}, stair_count});
    stair_edge[2].push_back({{x, y}, stair_count});
    stair_edge[3].push_back({{x, y}, stair_count});
}//end add_stair_edge_CoM

double StairClimb::add_stair_edge_CoMx(double x, double y) {  // only x relative to CoM, return x in wolrd coordinate
    stair_count ++;
    x += CoM[0];
    std::cout << "Add stair edge at (" << x << ", " << y << ") relative to origin." << std::endl;
    stair_edge[0].push_back({{x, y}, stair_count});
    stair_edge[1].push_back({{x, y}, stair_count});
    stair_edge[2].push_back({{x, y}, stair_count});
    stair_edge[3].push_back({{x, y}, stair_count});
    return x;  // return x in world coordinate
}//end add_stair_edge_CoM

void StairClimb::modify_last_edge_H(double y) {
    for (int i = 0; i < 4; i++) {
        if (!stair_edge[i].empty()) {
            stair_edge[i].back().edge[1] = y;
        } else {
            std::cout << "StairClimb::modify_last_edge_H: stair edge is empty." << std::endl;
        }//end if else
    }//end for
}//end modify_last_edge_H

void StairClimb::delete_last_edge() {
    for (int i = 0; i < 4; ++i) {
        if (!stair_edge[i].empty()) {
            stair_edge[i].pop_back();
        } else {
            std::cout << "Leg " << i << ": No stair edge to delete." << std::endl;
        }//end if else
    }//end for 

    stair_count--;
    std::cout << "Deleted last stair edge. Current count: " << stair_count << std::endl;
}//end delete_last_edge

double StairClimb::get_pitch() {
    return this->pitch;
}//end get_pitch

std::array<double, 2> StairClimb::get_CoM() {
    return this->CoM;
}//end get_CoM

bool StairClimb::if_any_stair() {
    for (int i=0; i<4; i++) {
        if (!stair_edge[i].empty()) {
            return true;
        }//end if
    }//end for
    return false;
}//end if_any_stair

bool StairClimb::any_no_stair() {
    for (int i=0; i<4; i++) {
        if (stair_edge[i].empty()) {
            return true;
        }//end if
    }//end for
    return false;
}//end any_no_stair

std::array<bool, 4> StairClimb::get_contact_edge_leg() {
    std::array<bool, 4> if_contact_edge = {false, false, false, false};
    for (int i=0; i<4; i++) {
        if (leg_info[i].contact_edge) {
            if_contact_edge[i] = true;
        }//end if
    }//end for
    return if_contact_edge;
}//end get_stair_count

double StairClimb::get_optimal_foothold(double H, bool get_front) {
    std::vector<std::pair<double, double>> foothold_table;
    if (get_front) {
        foothold_table = foothold_table_front;
    } else {
        foothold_table = foothold_table_hind;
    }//end if else

    if (H < foothold_table.front().first) {
        return foothold_table.front().second;
    } else if (H > foothold_table.back().first) {
        throw std::out_of_range("H 超出查表範圍");
    }//end if else

    // 找到 H 介於哪兩個點之間
    for (size_t i = 0; i < foothold_table.size() - 1; ++i) {
        double H1 = foothold_table[i].first;
        double V1 = foothold_table[i].second;
        double H2 = foothold_table[i + 1].first;
        double V2 = foothold_table[i + 1].second;
        if (H >= H1 && H <= H2) {
            // 線性內插公式
            double ratio = (H - H1) / (H2 - H1);
            return V1 + ratio * (V2 - V1);
        }//end if
    }//end for 
}//end get_optimal_foothold


/* Private function */
void StairClimb::init_move_CoM_stable(int swing_leg) { 
    this->swing_leg = swing_leg;
    this->move_dir = (swing_leg == 0 || swing_leg == 1) ? -1 : 1;
    this->CoM_offset = { std::cos(pitch) * CoM_bias[0] - std::sin(pitch) * CoM_bias[1],
                         std::sin(pitch) * CoM_bias[0] + std::cos(pitch) * CoM_bias[1] };
    this->achieve_max_length = false;
}//end init_move_CoM_stable

bool StairClimb::move_CoM_stable() {    // return true if stable, false if not
    this->update_hip();
    /* Change velocity */
    if (move_dir * velocity[0] < max_velocity) {
        velocity[0] += move_dir * vel_incre;
    } else if (move_dir * velocity[0] > max_velocity + vel_incre) {
        velocity[0] -= move_dir * vel_incre;
    }//end if else
    CoM[0] += velocity[0] / rate;
    /* Change to wheel mode when contact edge */
    if (move_dir == 1) {
        #if WHEEL_MODE_WHEN_CONTACT
        velocity[1] = 0;
        if (leg_info[0].stair_count != leg_info[1].stair_count) {
            if (leg_info[0].contact_edge || enter_wheel_mode[0]) {
                // if (last_hip[0][0] > stair_edge[0].front().edge[0]) { // front leg is further than edge
                    // enter_wheel_mode[0] = true; // enter wheel mode
                    double p_x = leg_info[1].foothold[0] - hip[1][0];   // x dir of hip to contact point
                    double p_y = leg_info[1].foothold[1] - hip[1][1] + leg_model.radius;   // y dir of hip to contact point + radius
                    velocity[1] = std::abs(p_x)>1e-4? velocity[0] / p_x * p_y : 0.0;
                    int sign_vel = velocity[1]>=0.0? 1 : -1;
                    if (std::abs(velocity[1]) > max_velocity) {
                        velocity[1] = sign_vel * max_velocity;
                    }//end if

                    double max_down = last_hip[0][1] - (stair_edge[0].front().edge[1] + leg_model.radius);
                    if (max_down > sign_vel*max_velocity / rate) {
                        front_height += velocity[1] / rate;
                    } else {
                        front_height -= max_down;
                        // if (theta[0]*180/M_PI < 17.1) {
                        if (true) {
                            wheel_mode[0] = true; // enter wheel mode
                        }//end if
                    }//end if else
                // }//end if
            } else if (leg_info[1].contact_edge || enter_wheel_mode[1]) {
                // if (last_hip[1][0] > stair_edge[1].front().edge[0]) { // front leg is further than edge
                    // enter_wheel_mode[1] = true; // enter wheel mode
                    double p_x = leg_info[0].foothold[0] - hip[0][0];   // x dir of hip to contact point
                    double p_y = leg_info[0].foothold[1] - hip[0][1] + leg_model.radius;   // y dir of hip to contact point + radius
                    velocity[1] = std::abs(p_x)>1e-4? velocity[0] / p_x * p_y : 0.0;
                    int sign_vel = velocity[1]>=0.0? 1 : -1;
                    if (std::abs(velocity[1]) > max_velocity) {
                        velocity[1] = sign_vel * max_velocity;
                    }//end if

                    double max_down = last_hip[1][1] - (stair_edge[1].front().edge[1] + leg_model.radius);
                    if (max_down > sign_vel*max_velocity / rate) {
                        front_height += velocity[1] / rate;
                    } else {
                        front_height -= max_down;
                        // if (theta[1]*180/M_PI < 17.1) {
                        if (true) {
                            wheel_mode[1] = true; // enter wheel mode
                        }//end if
                    }//end if else
                // }//end if
            }//end if else
        }//end if

        velocity[1] = 0;
        if (leg_info[2].stair_count != leg_info[3].stair_count) {
            if (leg_info[2].contact_edge || enter_wheel_mode[2]) {
                // if (last_hip[2][0] > stair_edge[2].front().edge[0]) { // hind leg is further than edge
                    // enter_wheel_mode[2] = true; // enter wheel mode
                    double p_x = leg_info[3].foothold[0] - hip[3][0];   // x dir of hip to contact point
                    double p_y = leg_info[3].foothold[1] - hip[3][1] + leg_model.radius;   // y dir of hip to contact point + radius
                    velocity[1] = std::abs(p_x)>1e-4? velocity[0] / p_x * p_y : 0.0;
                    int sign_vel = velocity[1]>=0.0? 1 : -1;
                    if (std::abs(velocity[1]) > max_velocity) {
                        velocity[1] = sign_vel * max_velocity;
                    }//end if

                    double max_down = last_hip[2][1] - (stair_edge[2].front().edge[1] + leg_model.radius);
                    if (max_down > sign_vel*max_velocity / rate) {
                        hind_height += velocity[1] / rate;
                    } else {
                        hind_height -= max_down;
                        // if (theta[2]*180/M_PI < 17.1) {
                        if (true) {
                            wheel_mode[2] = true; // enter wheel mode
                        }//end if
                    }//end if else
                // }//end if
            } else if (leg_info[3].contact_edge || enter_wheel_mode[3]) {
                // if (last_hip[3][0] > stair_edge[3].front().edge[0]) { // hind leg is further than edge
                    // enter_wheel_mode[3] = true; // enter wheel mode
                    double p_x = leg_info[2].foothold[0] - hip[2][0];   // x dir of hip to contact point
                    double p_y = leg_info[2].foothold[1] - hip[2][1] + leg_model.radius;   // y dir of hip to contact point + radius
                    velocity[1] = std::abs(p_x)>1e-4? velocity[0] / p_x * p_y : 0.0;
                    int sign_vel = velocity[1]>=0.0? 1 : -1;
                    if (std::abs(velocity[1]) > max_velocity) {
                        velocity[1] = sign_vel * max_velocity;
                    }//end if

                    double max_down = last_hip[3][1] - (stair_edge[3].front().edge[1] + leg_model.radius);
                    if (max_down > sign_vel*max_velocity / rate) {
                        hind_height += velocity[1] / rate;
                    } else {
                        hind_height -= max_down;
                        // if (theta[3]*180/M_PI < 17.1) {
                        if (true) {
                            wheel_mode[3] = true; // enter wheel mode
                        }//end if
                    }//end if else
                // }//end if
            }//end if else
        }//end if
        #else
        bool have_change_velocity = false;
        if (leg_info[0].stair_count != leg_info[1].stair_count) {
            velocity[1] = -velocity[0];

            if (leg_info[0].contact_edge) {
                if (last_hip[0][0] > stair_edge[0].front().edge[0]) { // front leg is further than edge
                    double max_down = last_hip[0][1] - (stair_edge[0].front().edge[1] + leg_model.radius);
                    if (max_down > - velocity[1] / rate) {
                        front_height += velocity[1] / rate;
                    } else if (max_down < velocity[1] / rate) {
                        front_height -= velocity[1] / rate;
                    } else {
                        front_height -= max_down;
                        if (theta[0]*180/M_PI < 17.1) {
                            wheel_mode[0] = true; // enter wheel mode
                        }//end if
                    }//end if else
                }//end if
            } else if (leg_info[1].contact_edge) {
                if (last_hip[1][0] > stair_edge[1].front().edge[0]) { // front leg is further than edge
                    double max_down = last_hip[1][1] - (stair_edge[1].front().edge[1] + leg_model.radius);
                    if (max_down > - velocity[1] / rate) {
                        front_height += velocity[1] / rate;
                    } else if (max_down < velocity[1] / rate) {
                        front_height -= velocity[1] / rate;
                    } else {
                        front_height -= max_down;
                        if (theta[1]*180/M_PI < 17.1) {
                            wheel_mode[1] = true; // enter wheel mode
                        }//end if
                    }//end if else
                }//end if
            }//end if else
        }//end if

        if (leg_info[2].stair_count != leg_info[3].stair_count) {
            velocity[1] = -velocity[0];

            if (leg_info[2].contact_edge) {
                if (last_hip[2][0] > stair_edge[2].front().edge[0]) { // hind leg is further than edge
                    double max_down = last_hip[2][1] - (stair_edge[2].front().edge[1] + leg_model.radius);
                    if (max_down > - velocity[1] / rate) {
                        hind_height += velocity[1] / rate;
                    } else if (max_down < velocity[1] / rate) {
                        hind_height -= velocity[1] / rate;
                    } else {
                        hind_height -= max_down;
                        if (theta[2]*180/M_PI < 17.1) {
                            wheel_mode[2] = true; // enter wheel mode
                        }//end if
                    }//end if else
                }//end if
            } else if (leg_info[3].contact_edge) {
                if (last_hip[3][0] > stair_edge[3].front().edge[0]) { // hind leg is further than edge
                    double max_down = last_hip[3][1] - (stair_edge[3].front().edge[1] + leg_model.radius);
                    if (max_down > - velocity[1] / rate) {
                        hind_height += velocity[1] / rate;
                    } else if (max_down < velocity[1] / rate) {
                        hind_height -= velocity[1] / rate;
                    } else {
                        hind_height -= max_down;
                        if (theta[3]*180/M_PI < 17.1) {
                            wheel_mode[3] = true; // enter wheel mode
                        }//end if
                    }//end if else
                }//end if
            }//end if else
        }//end if
        #endif
    }//end if
    CoM[1] = (front_height + hind_height) / 2; // update CoM height
    pitch = std::asin((front_height - hind_height) / BL); // update pitch angle
    /* Calculate leg command */
    for (int i=0; i<4; i++) {
        hip[i] = leg_info[i].get_hip_position(CoM, pitch);
        result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], hip[i][1]-last_hip[i][1]});
        theta[i] = result_eta[0];
        beta[i]  = result_eta[1];
    }//end for
    /* Return if stable (entering support triangle) */
    if (move_dir * (CoM[0] + CoM_offset[0]) > move_dir * ((leg_info[(swing_leg+1)%4].foothold[0] + leg_info[(swing_leg+3)%4].foothold[0]) / 2) + stability_margin) {
        velocity[1] = 0.0;
        return true;
    } else {
        return false;
    }//end if else
}//end move_CoM_stable

bool StairClimb::slow_to_stop() {    // return true if stop, false if not
    this->update_hip();
    /* Slow down velocity */
    if (velocity[0] > vel_incre) {
        velocity[0] -= vel_incre;
    } else if (velocity[0] < -vel_incre) {
        velocity[0] += vel_incre;
    } else {
        velocity[0] = 0.0;
    }//end if else
    CoM[0] += velocity[0] / rate;
    /* Calculate leg command */
    for (int i=0; i<4; i++) {
        hip[i] = leg_info[i].get_hip_position(CoM, pitch);
        result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], hip[i][1]-last_hip[i][1]});
        theta[i] = result_eta[0];
        beta[i]  = result_eta[1];
    }//end for
    /* Return if stop */
    if (velocity[0] == 0.0) {
        return true;
    } else {
        return false;
    }//end if else
}//end slow_to_stop

// void StairClimb::move_CoM_stable_smooth_fixed_leg_length(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch) {
//     Eigen::Matrix2d rotation;
//     rotation << std::cos(pitch), -std::sin(pitch),
//                 std::sin(pitch),  std::cos(pitch);
//     Eigen::Vector2d CoM_offset = rotation * CoM_bias;

//     int move_dir = (leg_info[swing_leg].ID == 0 || leg_info[swing_leg].ID == 1) ? -1 : 1;
//     LegModel leg_model;
//     leg_model.forward(0.0, 0.0); // 初始化腿部模型
//     double leg_length = leg_model.G.norm();
    
//     while (move_dir * (CoM[0] + CoM_offset[0]) < move_dir * ((leg_info[(swing_leg + 1) % 4].foothold[0] + leg_info[(swing_leg - 1) % 4].foothold[0]) / 2) + stability_margin) {
//         if (move_dir * velocity[0] < max_velocity) {
//             velocity[0] += move_dir * acc / sampling;
//         }//end if
//         CoM += velocity / sampling;
        
//         double hip_x = CoM[0];
//         double hip_y;
//         leg_model.forward(0.0, 0.0);
//         if (leg_info[swing_leg].contact_edge) {
//             hip_y = CoM[1];
//         } else {
//             hip_y = CoM[1] + leg_model.G[1] + std::sqrt(leg_length * leg_length - std::pow(hip_x - (CoM[0] + leg_model.G[0]), 2));
//         }//end if else
        
//         Eigen::Vector2d hip = Eigen::Vector2d(hip_x, hip_y);
//         std::cout << "Hip position for swing leg " << swing_leg << ": " << hip.transpose() << std::endl;
//     }//end while
//     CoM = Eigen::Vector2d(CoM[0], CoM[1]);
//     pitch = std::asin((CoM[1] - CoM[1]) / BL);
// }//end move_CoM_stable_fixed_leg_length

void StairClimb::init_swing_same_step(int swing_leg, double front_height, double hind_height) { 
    this->swing_leg = swing_leg;
    this->margin_d = std::abs(CoM[0] - (leg_info[(swing_leg+1)%4].foothold[0]+leg_info[(swing_leg+3)%4].foothold[0])/2) - stability_margin;
    leg_model.forward(theta[swing_leg], beta[swing_leg]);
    std::array<double, 2> p_lo = {hip[swing_leg][0] + leg_model.G[0], hip[swing_leg][1] + leg_model.G[1]}; 
    std::array<double, 2> p_td = {leg_info[swing_leg].next_foothold[0], leg_info[swing_leg].next_foothold[1]+leg_model.r};
    this->sp[swing_leg] = SwingProfile(p_lo, p_td, step_height, 1);

    this->final_CoM_height = (front_height + hind_height) / 2;
    this->coeff_b = acc[1];
    this->coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    this->t_f_x = margin_d / max_velocity;
    this->t_f_y = - coeff_b / coeff_a;
    this->t_f = std::max({min_swing_time_step, t_f_x, t_f_y});
    // this->local_max_velocity = margin_d / t_f;
    this->local_max_velocity = 0.0;
    this->total_steps = static_cast<int>(t_f * rate); // total steps for swinging
    this->step_count = 0;
}//end init_swing_same_step

bool StairClimb::swing_same_step() {  // return true if finish swinging, false if not
    this->update_hip();
    /* Change velocity */
    if (velocity[0] < local_max_velocity - vel_incre) {
        velocity[0] += vel_incre;
    } else if (velocity[0] > local_max_velocity + vel_incre) {
        velocity[0] -= vel_incre;
    } else {
        velocity[0] = local_max_velocity;
    }//end if else
    double t_ = (step_count+1.0) / rate;
    velocity[1] = (t_ <= t_f_y)? coeff_a * (t_*t_) + coeff_b * t_ : 0.0;
    CoM[0] += velocity[0] / rate;
    CoM[1] += velocity[1] / rate;
    /* Calculate pitch */
    if (swing_leg == 0 || swing_leg == 1) {
        pitch = std::asin((CoM[1]-hind_height) / (BL/2));
    } else {
        pitch = std::asin((front_height-CoM[1]) / (BL/2));
    }//end if else
    /* Calculate leg command */
    for (int i=0; i<4; i++) {
        hip[i] = leg_info[i].get_hip_position(CoM, pitch);
        if (i == swing_leg) {
            double swing_phase_ratio = (step_count+1.0) / total_steps;
            std::array<double, 2> curve_point = sp[i].getFootendPoint(swing_phase_ratio);
            std::array<double, 2> pos = {curve_point[0] - hip[i][0], curve_point[1] - hip[i][1]};
            result_eta = leg_model.inverse(pos, "G");
        } else {
            result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], hip[i][1]-last_hip[i][1]});
        }//end if else
        theta[i] = result_eta[0];
        beta[i]  = result_eta[1];
    }//end for
    step_count ++;
    /* Return if finish swinging */
    if (step_count==total_steps) {
        velocity[1] = 0.0;
        return true;
    } else {
        return false;
    }//end if else
}//end swing_same_step

void StairClimb::init_swing_next_step(int swing_leg, double front_height, double hind_height) { 
    this->swing_leg = swing_leg;
    // this->is_clockwise = (swing_leg == 0 || swing_leg == 1)? (stair_edge[0].front().count == stair_edge[1].front().count) : (stair_edge[2].front().count == stair_edge[3].front().count);

    this->final_CoM_height = (front_height + hind_height) / 2;
    this->coeff_b = acc[1];
    this->coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    this->t_f_x = velocity[0] / acc[0];
    this->t_f_y = - coeff_b / coeff_a;
    this->t_f = is_clockwise? std::max({min_swing_time_cw, t_f_x, t_f_y}) : std::max({min_swing_time_ccw, t_f_x, t_f_y});
    this->total_steps = static_cast<int>(t_f * rate); // total steps for swinging

    int sign_vel = velocity[0]>=0.0? 1 : -1;
    this->init_CoM = CoM;
    this->final_CoM = {CoM[0] + velocity[0]*t_f_x - sign_vel*acc[0]*(t_f_x*t_f_x)/2, final_CoM_height};
    this->init_pitch = pitch;
    this->final_pitch = std::asin((front_height-hind_height)/BL);
    this->init_front_height = hip[0][1];
    this->init_hind_height  = hip[3][1];
    this->final_hip = leg_info[swing_leg].get_hip_position(final_CoM, final_pitch);

    for (int i=0; i<3; i++) {
        double rim_radius = i==0? leg_model.r : leg_model.radius;
        std::array<double, 2> pos = { 
            leg_info[swing_leg].next_foothold[0] - final_hip[0],
            leg_info[swing_leg].next_foothold[1] - final_hip[1] + rim_radius
        };
        result_eta = leg_model.inverse(pos, touch_rim_list[i]);
        leg_model.contact_map(result_eta[0], result_eta[1]);
        if (std::find(touch_rim_idx[i].begin(), touch_rim_idx[i].end(), leg_model.rim) != touch_rim_idx[i].end()) {
            break;
        }//end if
    }//end for
    final_theta = result_eta[0];
    final_beta  = result_eta[1];

    this->first_ratio  = is_clockwise? 0.1 : 0.3;   // 0 ~ first_ratio by G vertical up
    this->second_ratio = is_clockwise? 0.7 : 0.7;   // first_ratio ~ mid_ratio by theta and beta
    this->first_in  = true;
    this->second_in = true;
    this->third_in  = true;
    this->step_count = 0;
}//end init_swing_same_step

bool StairClimb::swing_next_step() {  // return true if finish swinging, false if not
    this->update_hip();
    /* Change velocity */
    if (velocity[0] > vel_incre) {
        velocity[0] -= vel_incre;
    } else if (velocity[0] < -vel_incre) {
        velocity[0] += vel_incre;
    } else {
        velocity[0] = 0.0;
    }//end if else
    double t_ = (step_count+1.0) / rate;
    velocity[1] = (t_<=t_f_y) ? coeff_a * (t_*t_) + coeff_b * t_ : 0.0;
    CoM[0] += velocity[0] / rate;
    CoM[1] += velocity[1] / rate;
    /* Calculate pitch */
    double CoM_up_ratio = final_CoM[1]==init_CoM[1]? 0 : (CoM[1] - init_CoM[1]) / (final_CoM[1] - init_CoM[1]);
    double current_front_height = init_front_height + CoM_up_ratio*(front_height - init_front_height);
    double current_hind_height  = init_hind_height  + CoM_up_ratio*(hind_height - init_hind_height);
    pitch = std::asin((current_front_height-current_hind_height) / BL);
    /* Calculate leg command */
    for (int i=0; i<4; i++) {
        hip[i] = leg_info[i].get_hip_position(CoM, pitch);
        if (i == swing_leg) {
            double swing_phase_ratio = (step_count+1.0) / total_steps;
            /* Bezier curve */
            #if BEZIER_CURVE_SWING
            if (first_in) {
                first_in = false;
                leg_model.forward(theta[i], beta[i]);
                std::array<double, 2> current_G = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
                leg_model.forward(final_theta, final_beta);
                std::array<double, 2> final_G = {final_hip[0] + leg_model.G[0], final_hip[1] + leg_model.G[1]};
                this->sp[i] = SwingProfile(current_G, final_G, final_G[1]-current_G[1]+step_height, 1);
            }//end if
            std::array<double, 2> curve_point = sp[i].getFootendPoint(swing_phase_ratio);
            std::array<double, 2> pos = {curve_point[0] - hip[i][0], curve_point[1] - hip[i][1]};
            result_eta = leg_model.inverse(pos, "G");
            /* Clockwise swing (first swing leg) */
            #else
            if (swing_phase_ratio < first_ratio) {  // first phase
                if (first_in) {
                    first_in = false;
                    leg_model.forward(theta[i], beta[i]);
                    std::array<double, 2> current_G = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
                    para_traj[0] = LinearParaBlend({current_G[0], current_G[0]}            , {0.0, 1.0}, 0.3, true, 0.0, false, 0.0);
                    para_traj[1] = LinearParaBlend({current_G[1], current_G[1]+step_height}, {0.0, 1.0}, 0.3, true, 0.0, false, 0.0);
                }//end if
                if (is_clockwise) {
                    swing_phase_ratio = swing_phase_ratio / first_ratio;
                    double x_p = para_traj[0].get_point(swing_phase_ratio);
                    double y_p = para_traj[1].get_point(swing_phase_ratio);
                    std::array<double, 2> pos = {x_p-hip[i][0], y_p-hip[i][1]};
                    result_eta = leg_model.inverse(pos, "G");
                } else {
                    // result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], 0});
                    result_eta = {theta[i], beta[i]};
                }//end if else
                last_theta[i] = theta[i];
                last_beta[i]  = beta[i];
            } else if (swing_phase_ratio < second_ratio) {  // second phase
                if (second_in) {
                    second_in = false;
                    // last point & velocity of first phase
                    double v_theta = (theta[i] - last_theta[i]) * rate * ((second_ratio-first_ratio)*total_steps/rate);
                    double v_beta  = (beta[i]  - last_beta[i])  * rate * ((second_ratio-first_ratio)*total_steps/rate);
                    leg_model.forward(final_theta, final_beta);
                    std::array<double, 2> final_G = {final_hip[0] + leg_model.G[0], final_hip[1] + leg_model.G[1]};
                    if (is_clockwise || wheel_mode[i]) {
                        std::array<double, 2> C1 = final_hip;
                        std::array<double, 2> P1 = {final_G[0], final_G[1] + step_height};
                        std::array<double, 2> C1_P1 = {P1[0]-C1[0], P1[1]-C1[1]};
                        double C1_P1_d = std::hypot(C1_P1[0], C1_P1[1]);
                        if (C1_P1_d < leg_model.R) {
                            std::cout << "StairClimb::swing_next_step: P1 is inside the wheel." << std::endl;
                            P1 = {final_G[0], final_hip[1]-leg_model.R};
                            C1_P1 = {P1[0]-C1[0], P1[1]-C1[1]};
                            C1_P1_d = std::hypot(C1_P1[0], C1_P1[1]);
                        } //end if
                        double tan_line_angle = std::acos(leg_model.R / C1_P1_d);
                        double second_theta = 17.0/180.0*M_PI;
                        double second_beta  = std::atan2(C1_P1[1], C1_P1[0]) + tan_line_angle + M_PI/2 - 2*M_PI;
                        para_traj[0] = LinearParaBlend({theta[i], second_theta             , second_theta}, {0.0, 0.5, 1.0}, 0.1, true, v_theta, false, 0.0);
                        para_traj[1] = LinearParaBlend({beta[i] , (beta[i]+second_beta)/2.0, second_beta }, {0.0, 0.5, 1.0}, 0.1, true, v_beta , false, 0.0);
                    } else {
                        leg_model.forward(theta[i], beta[i]);
                        std::array<double, 2> current_G = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
                        this->sp[i] = SwingProfile(current_G, final_G, final_G[1]-current_G[1]+2*step_height, 1);
                    }//end if else
                }//end if
                if (is_clockwise || wheel_mode[i]) {
                    swing_phase_ratio = (swing_phase_ratio - first_ratio) / (second_ratio - first_ratio);
                    result_eta[0] = para_traj[0].get_point(swing_phase_ratio);
                    result_eta[1] = para_traj[1].get_point(swing_phase_ratio);
                } else {
                    swing_phase_ratio = (swing_phase_ratio - first_ratio) / (1.0 - first_ratio);
                    std::array<double, 2> curve_point = sp[i].getFootendPoint(swing_phase_ratio);
                    std::array<double, 2> pos = {curve_point[0] - hip[i][0], curve_point[1] - hip[i][1]};
                    result_eta = leg_model.inverse(pos, "G");
                }//end if else
                last_theta[i] = theta[i];
                last_beta[i]  = beta[i];
                for (int j=0; j<4; j++) {
                    last2_hip[j][0] = last_hip[j][0];
                    last2_hip[j][1] = last_hip[j][1];
                }//end for
            } else {    // third phase
                if (third_in) {
                    third_in = false;
                    // last point & velocity of second phase
                    leg_model.forward(theta[i], beta[i]);
                    std::array<double, 2> last_G = {last_hip[i][0] + leg_model.G[0], last_hip[i][1] + leg_model.G[1]};
                    leg_model.forward(last_theta[i], last_beta[i]);
                    std::array<double, 2> last2_G = {last2_hip[i][0] + leg_model.G[0], last2_hip[i][1] + leg_model.G[1]};
                    double v_x = (last_G[0] - last2_G[0]) * rate * ((1.0-second_ratio)*total_steps/rate);
                    double v_y = (last_G[1] - last2_G[1]) * rate * ((1.0-second_ratio)*total_steps/rate);
                    leg_model.forward(final_theta, final_beta);
                    std::array<double, 2> final_G = {final_hip[0] + leg_model.G[0], final_hip[1] + leg_model.G[1]};
                    para_traj[0] = LinearParaBlend({last_G[0], final_G[0]            , final_G[0]}, {0.0, 0.5, 1.0}, 0.3, true, v_x, true, 0.0);
                    para_traj[1] = LinearParaBlend({last_G[1], final_G[1]+step_height, final_G[1]}, {0.0, 0.5, 1.0}, 0.3, true, v_y, true, 0.0);
                }//end if
                swing_phase_ratio = (swing_phase_ratio - second_ratio) / (1.0 - second_ratio);
                double x_p = para_traj[0].get_point(swing_phase_ratio);
                double y_p = para_traj[1].get_point(swing_phase_ratio);
                std::array<double, 2> pos = {x_p-hip[i][0], y_p-hip[i][1]};
                result_eta = leg_model.inverse(pos, "G");
            }//end if else
            #endif
        } else {
            result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], hip[i][1]-last_hip[i][1]});
        }//end if else
        theta[i] = result_eta[0];
        beta[i]  = result_eta[1];
    }//end for
    step_count ++;
    /* Return if finish swinging */
    if (step_count==total_steps) {
        velocity[1] = 0.0;
        return true;
    } else {
        return false;
    }//end if else
}//end swing_next_step

std::array<double, 2> StairClimb::move_consider_edge(int leg_ID, std::array<double, 2> move_vec) {
    std::array<double, 2> current_stair_edge = {INFINITY, 0}; // [0] set very large to ensure execute correctly when the vector is empty.
    leg_model.forward(theta[leg_ID], beta[leg_ID]);
    if (!stair_edge[leg_ID].empty()) {
        current_stair_edge = stair_edge[leg_ID].front().edge;
        double err = 0.01;
        
        std::array<double, 2> edge_U_vec = {hip[leg_ID][0]+leg_model.U_r[0]-current_stair_edge[0], hip[leg_ID][1]+leg_model.U_r[1]-current_stair_edge[1]};
        if (!leg_info[leg_ID].contact_edge && (hip[leg_ID][0]+leg_model.U_r[0] <= current_stair_edge[0]) && (std::hypot(edge_U_vec[0], edge_U_vec[1]) <= leg_model.radius)) {
            leg_info[leg_ID].contact_edge = true;
            leg_model.forward(theta[leg_ID], beta[leg_ID], false);
            std::complex<double> current_stair_edge_c(current_stair_edge[0], current_stair_edge[1]);
            std::complex<double> hip_c(hip[leg_ID][0], hip[leg_ID][1]);
            leg_info[leg_ID].contact_alpha = std::arg((current_stair_edge_c - hip_c - leg_model.U_r_c) / (leg_model.F_r_c-leg_model.U_r_c));
        } else if (leg_info[leg_ID].contact_edge && (hip[leg_ID][0]+leg_model.U_r[0]>current_stair_edge[0] || std::hypot(edge_U_vec[0], edge_U_vec[1]) > leg_model.radius + err)) {
            leg_info[leg_ID].contact_edge = false;
            achieve_max_length = false;
        }//end if else
    }//end if
    
    if (leg_info[leg_ID].contact_edge) {
        result_eta = move_edge(leg_ID, {current_stair_edge[0]-hip[leg_ID][0], current_stair_edge[1]-hip[leg_ID][1]}, leg_info[leg_ID].contact_alpha);
        leg_info[leg_ID].foothold = {current_stair_edge[0], current_stair_edge[1]};
    } else {
        std::array<double, 2> relative_foothold;
        if (hip[leg_ID][0] + leg_model.U_r[0] > current_stair_edge[0]) {
            // if (theta[leg_ID]*180/M_PI < 17.1 && move_vec[1]==0.0) {    // wheel mode
            if (wheel_mode[leg_ID]) {    // wheel mode
                result_eta[0] = 17.0/180.0*M_PI;
                result_eta[1] = beta[leg_ID] - move_vec[0]/leg_model.radius;
                relative_foothold = {0.0, -leg_model.radius};
            } else {    // upper rim 
                result_eta = leg_model.move(theta[leg_ID], beta[leg_ID], move_vec, 0.0, true, false);
                relative_foothold = get_foothold(theta[leg_ID], beta[leg_ID], true, false);
            }//end if else
        } else {    // lower rim
            if (beta[leg_ID] < 0.0) {   // may contact using right upper rim
                result_eta = leg_model.move(theta[leg_ID], beta[leg_ID], move_vec, 0.0);
                relative_foothold = get_foothold(theta[leg_ID], beta[leg_ID]);
            } else {    // lower rim / G
                result_eta = leg_model.move(theta[leg_ID], beta[leg_ID], move_vec, 0.0, false);
                relative_foothold = get_foothold(theta[leg_ID], beta[leg_ID], false);
            }//end if else
        }//end if else
        leg_info[leg_ID].foothold = {hip[leg_ID][0] + relative_foothold[0], hip[leg_ID][1] + relative_foothold[1]};
    }//end if else


    if (std::abs(result_eta[0]-theta[leg_ID])>0.1 || std::abs(result_eta[1]-beta[leg_ID])>0.1) {
        result_eta[0] = theta[leg_ID];
        result_eta[1] = beta[leg_ID];
    }
    return result_eta;
}//end move_consider_edge

std::array<double, 2> StairClimb::move_edge(int leg_ID, std::array<double, 2> contact_p, double contact_alpha, double tol, size_t max_iter) {
    leg_model.forward(theta[leg_ID], beta[leg_ID]);
    std::array<double, 2> init_U = leg_model.U_r;
    
    // Use optimization solver to find d_theta and d_beta (analogous to fsolve)
    std::array<double, 2> guess_dq = {0.0, 0.0};    // d_theta, d_beta / initial guess = (0, 0)
    for (size_t iter = 0; iter < max_iter; ++iter) {
        std::array<double, 2> cost = this->objective_edge(guess_dq, {theta[leg_ID], beta[leg_ID]}, contact_p, contact_alpha);     // 计算当前函数值
        Eigen::Vector2d cost_vec(cost[0], cost[1]);
        
        double norm_cost = cost_vec.norm();          // 计算残差范数
        if (norm_cost < tol) {                // 判断收敛
            // std::cout << "cost converged after " << iter << " iterations.\n";
            break;
        }//end if

        // computeJacobian, 数值计算雅可比矩阵
        double epsilon = 1e-6;
        Eigen::Matrix2d Jac;
        for (size_t i = 0; i < 2; ++i) {
            std::array<double, 2> dq_eps = guess_dq;
            dq_eps[i] += epsilon;  // 对第 i 个变量加一个小扰动
            std::array<double, 2> cost_eps = this->objective_edge(dq_eps, {theta[leg_ID], beta[leg_ID]}, contact_p, contact_alpha);
            Eigen::Vector2d cost_eps_vec(cost_eps[0], cost_eps[1]);
            Jac.col(i) = (cost_eps_vec - cost_vec) / epsilon;  // 数值差分计算导数
        }//end for

        Eigen::Vector2d dq = Jac.partialPivLu().solve(-cost_vec);   // 解线性方程 Jac * dq = -cost_vec
        if (dq.norm() < epsilon) {             // 判断步长是否足够小
            // std::cout << "dx converged after " << iter << " iterations.\n";
            break;
        }//end if

        // 更新解
        guess_dq[0] += dq[0];
        guess_dq[1] += dq[1];

        if (iter == max_iter-1) {
            // throw std::runtime_error("StairClimb::Move_edge: Newton solver did not converge.");
            std::cout << "StairClimb::Move_edge: Newton solver cost " << norm_cost << std::endl;
            guess_dq[0] = 0.0;
            guess_dq[1] = 0.0;
        }//end if
    }//end for

    if (std::abs(guess_dq[0])>0.1 || std::abs(guess_dq[1])>0.1) {
        guess_dq[0] = 0.0;
        guess_dq[1] = 0.0;
    }
    result_eta[0] = theta[leg_ID] + guess_dq[0];
    result_eta[1] = beta[leg_ID]  + guess_dq[1];
    return result_eta;
}//end move_edge

std::array<double, 2> StairClimb::objective_edge(const std::array<double, 2>& d_q, const std::array<double, 2>& current_q, std::array<double, 2> contact_p, double contact_alpha) {
    std::array<double, 2> guessed_q = {current_q[0] + d_q[0], current_q[1] + d_q[1]};
    leg_model.forward(guessed_q[0], guessed_q[1]);

    std::complex<double> UF_complex(leg_model.F_r[0] - leg_model.U_r[0], leg_model.F_r[1] - leg_model.U_r[1]);
    std::complex<double> UP_exp = (UF_complex * std::exp(std::complex<double>(0, contact_alpha))) / leg_model.R * leg_model.radius;  // P: contact point
    std::array<double, 2> guessed_contact_p = {leg_model.U_r[0] + UP_exp.real(), leg_model.U_r[1] + UP_exp.imag()};
    
    return {guessed_contact_p[0] - contact_p[0], guessed_contact_p[1] - contact_p[1]};
}//end objective_edge



// std::array<double, 2> StairClimb::move_edge(int leg_ID, std::array<double, 2> contact_p, double contact_alpha, double tol, size_t max_iter) {
//     leg_model.forward(theta[leg_ID], beta[leg_ID]);
//     std::array<double, 2> init_U = leg_model.U_r;
    
//     // Use optimization solver to find d_x and d_y of init_U (analogous to fsolve)
//     double guess_da = 0.0;    // initial guess = 0, ds is displacement on the circle (center: edge, radius: leg_model.radius)
//     for (size_t iter = 0; iter < max_iter; ++iter) {
//         double cost = this->objective_edge(guess_da, init_U, contact_p, contact_alpha);     // 计算当前函数值
//         if (std::abs(cost) < tol) {                // 判断收敛
//             // std::cout << "cost converged after " << iter << " iterations.\n";
//             break;
//         }//end if

//         // computeJacobian, 数值计算雅可比矩阵
//         double epsilon = 1e-6;
//         double da_eps = guess_da + epsilon;
//         double cost_eps = this->objective_edge(da_eps, init_U, contact_p, contact_alpha);
//         double cost_d = (cost_eps - cost) / epsilon;  // 数值差分计算导数

//         double da = -cost / cost_d;   // 解线性方程 cost_d * dx = -cost
//         if (std::abs(da) < tol) {             // 判断步长是否足够小
//             // std::cout << "dx converged after " << iter << " iterations.\n";
//             break;
//         }//end if

//         // 更新解
//         guess_da += da;

//         if (iter == max_iter-1) {
//             throw std::runtime_error("StairClimb::Move_edge: Newton solver did not converge.");
//         }//end if
//     }//end for

//     double angle_0 = std::atan2(init_U[1] - contact_p[1], init_U[0] - contact_p[0]);
//     double new_angle = angle_0 + guess_da;
//     std::array<double, 2> new_U = {contact_p[0]+leg_model.radius*std::cos(new_angle), contact_p[1]+leg_model.radius*std::sin(new_angle)};
//     result_eta = leg_model.inverse(new_U, "U_r");
//     return result_eta;
// }//end move_edge

// double StairClimb::objective_edge(double d_a, std::array<double, 2> init_U, std::array<double, 2> contact_p, double contact_alpha) {
//     double angle_0 = std::atan2(init_U[1] - contact_p[1], init_U[0] - contact_p[0]);
//     double new_angle = angle_0 + d_a;
//     std::array<double, 2> new_U = {contact_p[0]+leg_model.radius*std::cos(new_angle), contact_p[1]+leg_model.radius*std::sin(new_angle)};
//     std::array<double, 2> new_result_eta = leg_model.inverse(new_U, "U_r");
//     leg_model.forward(new_result_eta[0], new_result_eta[1], false);

//     std::complex<double> contact_p_c(contact_p[0], contact_p[1]);
//     double new_alpha = std::arg((contact_p_c - leg_model.U_r_c) / (leg_model.F_r_c - leg_model.U_r_c));
//     return new_alpha - contact_alpha;
// }//end objective_edge

bool StairClimb::determine_next_foothold() {
    bool up_stair = false;
    std::array<double, 2> current_stair_edge;
    int current_stair_count = leg_info[swing_leg].stair_count;
    int next_stair = 0;
    double keep_stair_d_max = 0.0;
    double keep_stair_d_min = 0.18680;

    if (!stair_edge[swing_leg].empty()) {
        current_stair_edge  = stair_edge[swing_leg].front().edge;
        // if ((leg_info[swing_leg].next_up || leg_info[other_side_leg[swing_leg][1]].next_up) &&
        // (swing_leg < 2 || leg_info[other_side_leg[swing_leg][0]].one_step || current_stair_count + 1 < leg_info[other_side_leg[swing_leg][0]].stair_count)) {
        if ((leg_info[swing_leg].next_up || leg_info[other_side_leg[swing_leg][1]].next_up) &&
            (swing_leg < 2 || (leg_info[other_side_leg[swing_leg][0]].foothold[0] >= current_stair_edge[0] + keep_edge_d + min_foothold_distance) || current_stair_count + 1 < leg_info[other_side_leg[swing_leg][0]].stair_count)) {
            leg_info[swing_leg].next_up = false;
            if (current_stair_count == leg_info[other_side_leg[swing_leg][1]].stair_count) {    //first swing leg 
                leg_info[swing_leg].one_step = false;
                leg_info[swing_leg].next_foothold = {current_stair_edge[0] + keep_edge_d, current_stair_edge[1]};
            } else {    //second swing leg: the same as first swing leg unless the leg can not reach keep_stair_d_max (because change first swing leg at each stair)
                #if CHANGE_FIRST_SWING_LEG
                // if (swing_leg < 2) {    // front leg: change first swing leg
                if (true) {    // front leg: change first swing leg
                    double deepest_x;
                    double next_max_foothold_x = leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length_up_stair / 2;
                    if (stair_edge[swing_leg].size() >= 2) {
                        std::array<double, 2> next_stair_edge = stair_edge[swing_leg][1].edge;
                        double H = next_stair_edge[1] - current_stair_edge[1];
                        if (H > 0.01) {
                            double optimal_foothold = get_optimal_foothold(H, false);
                            keep_stair_d_max = optimal_foothold;
                            keep_stair_d_min = optimal_foothold;
                        }//end if
                        deepest_x = stair_edge[swing_leg][1].edge[0] - keep_stair_d_min - step_length_up_stair / 2;
                        // deepest_x = stair_edge[swing_leg][1].edge[0] - keep_stair_d_min;
                        // double middle_foothold = (deepest_x + leg_info[other_side_leg[swing_leg][1]].foothold[0]) / 2;
                        // deepest_x = std::min({middle_foothold, next_max_foothold_x});
                    } else {
                        deepest_x = INFINITY;
                        // deepest_x = next_max_foothold_x;
                    }//end if else
                    if (next_max_foothold_x >= deepest_x) {
                        // leg_info[swing_leg].next_foothold =  leg_info[other_side_leg[swing_leg][1]].foothold;
                        double next_larger_foothold = std::max({deepest_x, leg_info[other_side_leg[swing_leg][1]].foothold[0]});
                        leg_info[swing_leg].next_foothold =  {next_larger_foothold, current_stair_edge[1]};
                    } else {
                        leg_info[swing_leg].next_foothold = {next_max_foothold_x, current_stair_edge[1]};
                    }//end if else
                    leg_info[swing_leg].one_step = false;
                    // if (leg_info[swing_leg].next_foothold[0] > current_stair_edge[0] + step_length_up_stair / 2) {
                    //     leg_info[swing_leg].one_step = true;
                    // } else {
                    //     leg_info[swing_leg].one_step = false;
                    // }//end if else
                } else {    // hind leg: no change first swing leg
                    leg_info[swing_leg].one_step = true;
                    double deepest_x;
                    if (stair_edge[swing_leg].size() >= 2) {
                        deepest_x = stair_edge[swing_leg][1].edge[0] - keep_stair_d_min;
                    } else {
                        deepest_x = INFINITY;
                    }//end if else
                    double next_max_foothold_x = leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length_up_stair / 2;
                    if (next_max_foothold_x >= deepest_x) {
                        leg_info[swing_leg].next_foothold = {deepest_x, current_stair_edge[1]};
                    } else {
                        double middle_foothold = (deepest_x + leg_info[other_side_leg[swing_leg][1]].foothold[0]) / 2;
                        double next_smaller_foothold = std::min({middle_foothold, next_max_foothold_x});
                        leg_info[swing_leg].next_foothold = {next_smaller_foothold, current_stair_edge[1]};
                        // leg_info[swing_leg].next_foothold = {next_max_foothold_x, current_stair_edge[1]};
                    }//end if else
                }//end if else
                #else
                leg_info[swing_leg].one_step = true;
                double deepest_x;
                if (stair_edge[swing_leg].size() >= 2) {
                    deepest_x = stair_edge[swing_leg][1].edge[0] - keep_stair_d_min;
                } else {
                    deepest_x = INFINITY;
                }//end if else
                double next_max_foothold_x = leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length_up_stair / 2;
                if (next_max_foothold_x >= deepest_x) {
                    leg_info[swing_leg].next_foothold = {deepest_x, current_stair_edge[1]};
                } else {
                    leg_info[swing_leg].next_foothold = {next_max_foothold_x, current_stair_edge[1]};
                }//end if else
                #endif
            }//end if else
            up_stair = true;
            next_stair = 1;
        } else {    // move on the same stair step
            double H = current_stair_edge[1] - last_stair_edge[swing_leg].edge[1];
            double optimal_foothold = get_optimal_foothold(H, false);
            keep_stair_d_max = optimal_foothold;
            keep_stair_d_min = optimal_foothold;
            leg_info[swing_leg].one_step = true;
            double deepest_x = current_stair_edge[0] - keep_stair_d_min;
            double next_max_foothold_x = leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length / 2;
            if (next_max_foothold_x >= deepest_x) {
                leg_info[swing_leg].next_foothold = {deepest_x, leg_info[swing_leg].foothold[1]};
            } else {
                // leg_info[swing_leg].next_foothold = {next_max_foothold_x, leg_info[swing_leg].foothold[1]};
                double middle_foothold = (deepest_x + leg_info[other_side_leg[swing_leg][1]].foothold[0]) / 2;
                double next_smaller_foothold = std::min({middle_foothold, next_max_foothold_x});
                leg_info[swing_leg].next_foothold = {next_smaller_foothold, leg_info[swing_leg].foothold[1]};
            }//end if else
            if (swing_leg < 2) {
                double at_least_foothold = std::max({leg_info[swing_leg].next_foothold[0], last_stair_edge[swing_leg].edge[0] + keep_edge_d + min_foothold_distance});
                leg_info[swing_leg].next_foothold[0] = at_least_foothold;
            }//end if 
        }//end if
        // determine if next swing leg will swing up to next stair step
        if (stair_edge[swing_leg].size() > next_stair && leg_info[swing_leg].next_foothold[0] >= stair_edge[swing_leg][next_stair].edge[0] - keep_stair_d_max) {
            leg_info[swing_leg].next_up = true;
        }//end if
    } else {    // move on the upper ground
        leg_info[swing_leg].one_step = true;
        leg_info[swing_leg].next_foothold = {leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length / 2, leg_info[swing_leg].foothold[1]};
        std::cout << "Leg " << swing_leg << " is on the upper ground." << std::endl;
    }//end if else

    return up_stair;
}//end determine_next_foothold

std::array<double, 2> StairClimb::get_foothold(double theta, double beta, bool contact_upper, bool contact_lower) {
    leg_model.contact_map(theta, beta, 0.0, contact_upper, contact_lower);
    if (leg_model.rim != 0) {
        return leg_model.contact_p;
    } else {
        std::cerr << "StairClimb::get_foothold: the leg doesn't contact ground." << std::endl;
        return {0, 0};
    }//end if else

    // double radius = leg_model.radius;
    // std::complex<double> center_beta0;
    // switch (contact_rim) {
    //     case 1: // left upper rim
    //         center_beta0 = std::complex<double>(U_l_poly[0](theta), U_l_poly[1](theta));
    //         break;
    //     case 2: // left lower rim
    //         center_beta0 = std::complex<double>(L_l_poly[0](theta), L_l_poly[1](theta));
    //         break;
    //     case 3: // G
    //         center_beta0 = std::complex<double>(0.0, G_poly[1](theta));
    //         radius = leg_model.r;
    //         break;
    //     case 4: // right lower rim
    //         center_beta0 = std::complex<double>(L_r_poly[0](theta), L_r_poly[1](theta));
    //         break;
    //     case 5: // right upper rim
    //         center_beta0 = std::complex<double>(U_r_poly[0](theta), U_r_poly[1](theta));
    //         break;
    //     default:
    //         std::cerr << "ERROR IN get_foothold, contact_rim should be 1~5." << std::endl;
    //         return {0, 0};
    // }//end switch

    // std::complex<double> center_exp = center_beta0 * std::exp(std::complex<double>(0, beta));
    // return {center_exp.real(), center_exp.imag() - radius};
}//end get_foothold

void StairClimb::update_hip() { // set last hip position as current hip position
    for (int i=0; i<4; i++) {
        last_hip[i] = hip[i];
    }//end for
}//end update_hip
