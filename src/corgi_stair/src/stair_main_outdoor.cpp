#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <Eigen/Dense>

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "corgi_msgs/StairPlanes.h"
#include "leg_model.hpp"
#include "bezier.hpp"
#include "walk_gait.hpp"
#include "stair_climb.hpp"

#define INIT_THETA (M_PI*17.0/180.0)
#define INIT_BETA (0.0)

corgi_msgs::TriggerStamped trigger_msg;
corgi_msgs::StairPlanes plane_msg;

void trigger_cb(const corgi_msgs::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb

void camera_cb(const corgi_msgs::StairPlanes::ConstPtr& msg) {
    plane_msg = *msg;
}//end camera_cb

double getPitchFromTransform(const geometry_msgs::TransformStamped& tf) {
    tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // std::cout << "Pitch: " << pitch << " radians" << std::endl;
    return pitch;
}//end getPitchFromTransform


int main(int argc, char** argv) {
    ros::init(argc, argv, "stair_climb");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1, trigger_cb);
    ros::Subscriber camera_sub = nh.subscribe<corgi_msgs::StairPlanes>("stair_planes", 1, camera_cb);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    // pid command
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp_r = 150.0;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 150.0;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 
    std::ofstream stair_size_csv("stair_size.csv");
    stair_size_csv << "Time," << "Trigger," << "D," << "H," << "\n";
    std::ofstream command_pitch_CoM("command_pitch_CoM.csv");
    command_pitch_CoM << "Time," << "Trigger," << "pitch," << "CoM_x," << "CoM_z," << "\n";

    /* Setting variable */
    enum STATES {INIT, TRANSFORM, WAIT, WALK, STAIR, UPPER_WALK, END};
    const std::array<double, 2> CoM_bias = {0.0, 0.0};
    const std::array<double, 3> camera_bias = {-0.01, 0.25, 0.032};   // initial camera position in map frame, (x, y, z)
    const std::array<double, 2> CoM2cemera = {0.32075, 0.099};  // translation from CoM to camera
    const int sampling_rate = 1000;
    const int transform_count = 5*sampling_rate; // 5 second
    double init_eta[8] = {1.857467698281913, 0.4791102940603915, 1.6046663223045279, 0.12914729012802004, 1.6046663223045279, -0.12914729012802004, 1.857467698281913, -0.4791102940603915};  // stand height 0.25, step length 0.3
    double velocity = 0.1; // velocity for walk gait
    double stand_height = 0.25; // stand height for walk gait
    const double step_length = 0.3; // step length for walk gait and stair gait
    const double step_height = 0.06; // step height for walk gait
    const double max_step_length = 0.3;
    const double min_step_length = 0.2;
    const double hip_x = 0.222; // front hip x position in robot frame

    /* Initial variable */
    ros::Rate rate(sampling_rate);
    WalkGait walk_gait(false, CoM_bias[0], sampling_rate);
    StairClimb stair_climb(false, CoM_bias, sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA , INIT_BETA , INIT_BETA , INIT_BETA }}};   // init eta (wheel mode)
    
    /* Other variable */
    STATES state = INIT, last_state = INIT;
    bool first_camera_pose;
    double transform_ratio;
    bool trigger;
    int stair_count;
    int plane_count;
    int command_count;
    double pitch;
    std::array<double, 2> CoM = {0.0, 0.0}; // CoM position in map frame
    double max_cal_time = 0.0;
    int touchdown_leg;
    double to_stair_d, to_enter_d;
    std::array<bool, 4> if_contact_edge, last_if_contact_edge;
    geometry_msgs::TransformStamped camera_transform;
    double D_sum, H_sum, last_D_sum;
    std::array<int, 4> step_count;
    double optimal_foothold;
    int min_steps, max_steps;
    bool change_step_length;
    std::vector<double> H_vector;
    bool need_new_H;
    bool add_stair_edge = true; // flag to add stair edge

    /* Behavior loop */
    auto start = std::chrono::high_resolution_clock::now();
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);
    while (ros::ok()) {
        auto one_loop_start = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
        if (state == END) {
            break;
        }//end if
        /* State machine */
        switch (state) {
            case INIT:
                first_camera_pose = false;
                transform_ratio = 0.0;
                trigger = false;
                pitch = 0.0;
                stair_count = 0;
                plane_count = 0;
                command_count = 0;
                D_sum = 0.0;
                H_sum = 0.0;
                need_new_H = false;
                add_stair_edge = true;
                break;
            case TRANSFORM:
                transform_ratio += 1.0 / transform_count;
                for (int i=0; i<4; i++) {
                    eta_list[0][i] = INIT_THETA + transform_ratio * (init_eta[i*2]   - INIT_THETA);
                    eta_list[1][i] = INIT_BETA  + transform_ratio * (init_eta[i*2+1] - INIT_BETA);
                    eta_list[1][i] = (i == 1 || i == 2)? eta_list[1][i] : -eta_list[1][i];  
                }//end for
                break;
            case WAIT:
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    walk_gait.initialize(current_eta);
                }//end if
                break;
            case WALK:
                /* Get camera pose */
                if (tfBuffer.canTransform("map", "zedxm_camera_center", ros::Time(0), ros::Duration(0.0))) {
                    try {
                        camera_transform = tfBuffer.lookupTransform("map", "zedxm_camera_center", ros::Time(0));
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN_THROTTLE(1.0, "TF lookup failed even after canTransform: %s", ex.what());
                    }
                } else {
                    ROS_WARN_THROTTLE(1.0, "TF not available at this moment");
                }
                /* Add H size */
                if (plane_msg.horizontal.size() >= plane_count+2) {
                    H_vector.push_back(plane_msg.horizontal[plane_count+1] - plane_msg.horizontal[plane_count]);
                    plane_count++;
                }//end if
                /* Adjust last step length of walk gait */
                change_step_length = false;
                to_stair_d = hip_x + 100;  // set far from hip if not detect stair
                optimal_foothold = 0;
                if (plane_msg.vertical.size() >= 1 && H_vector.size() >= 1) {
                    Eigen::Vector3d camera_position(
                        camera_transform.transform.translation.x,
                        camera_transform.transform.translation.y,
                        camera_transform.transform.translation.z
                    );
                    Eigen::Vector3d normal_vec(
                        plane_msg.v_normal.x,
                        plane_msg.v_normal.y,
                        plane_msg.v_normal.z
                    );

                    double H = H_vector[0];
                    optimal_foothold = stair_climb.get_optimal_foothold(H, false);
                    to_stair_d = plane_msg.vertical[0] - camera_position.dot(normal_vec) + CoM2cemera[0]; // distance from robot center to stair edge
                    to_enter_d = to_stair_d - hip_x - optimal_foothold -(max_step_length/2); // Remaining distance to the gait change point
                    min_steps = static_cast<int>(std::ceil(to_enter_d / (max_step_length/2)));   // max step length = 30cm
                    max_steps = static_cast<int>(std::floor(to_enter_d / (min_step_length/2)));  // min step length = 20cm 
                    if (min_steps <= max_steps) {
                        change_step_length = true;
                        walk_gait.set_step_length(to_enter_d / (min_steps/2.0)); 
                    }
                }//end if
                eta_list = walk_gait.step();
                touchdown_leg = walk_gait.get_touchdown_leg();
                if (change_step_length && (touchdown_leg == 2 || touchdown_leg == 3)) { // walk_gait apply new step_length
                    std::cout << "step_length:" << to_enter_d / (min_steps/2.0) << std::endl;
                }
                CoM[0] += velocity / sampling_rate; // expected CoM x position
                command_pitch_CoM << ros::Time::now() << "," << (int)trigger_msg.enable << "," << 0.0 << "," << CoM[0] << "," << stand_height << "\n";
                command_count ++;
                break;
            case STAIR:
                /* Initialize stair-climbing mode */
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    stair_climb.initialize(current_eta, velocity, CoM[0]);
                }//end if
                /* Get camera pose */
                if (tfBuffer.canTransform("map", "zedxm_camera_center", ros::Time(0), ros::Duration(0.0))) {
                    try {
                        camera_transform = tfBuffer.lookupTransform("map", "zedxm_camera_center", ros::Time(0));
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN_THROTTLE(1.0, "TF lookup failed even after canTransform: %s", ex.what());
                    }
                } else {
                    ROS_WARN_THROTTLE(1.0, "TF not available at this moment");
                }
                /* Add H size */
                if (plane_msg.horizontal.size() >= plane_count+2) {
                    H_vector.push_back(plane_msg.horizontal[plane_count+1] - plane_msg.horizontal[plane_count]);
                    plane_count++;
                }//end if
                /* Add stair edge */
                if (stair_climb.any_no_stair()) {
                    if ( (plane_msg.vertical.size() >= stair_count+1) && add_stair_edge) {
                        Eigen::Vector3d camera_position(
                            camera_transform.transform.translation.x,
                            camera_transform.transform.translation.y,
                            camera_transform.transform.translation.z
                        );
                        Eigen::Vector3d normal_vec(
                            plane_msg.v_normal.x,
                            plane_msg.v_normal.y,
                            plane_msg.v_normal.z
                        );

                        double next_edge_x = plane_msg.vertical[stair_count] - camera_position.dot(normal_vec);     // relative to camera
                        double real_pitch = pitch;
                        std::cout << "Pitch: " << real_pitch << " radians" << std::endl;
                        double CoM2camera_x = CoM2cemera[0] * std::cos(real_pitch) - CoM2cemera[1] * std::sin(real_pitch);
                        double CoM2camera_z = CoM2cemera[0] * std::sin(real_pitch) + CoM2cemera[1] * std::cos(real_pitch);
                        
                        last_D_sum = D_sum;  // save last D_sum
                        D_sum = stair_climb.add_stair_edge_CoMx(CoM2camera_x + next_edge_x, H_sum);
                        double D = D_sum - last_D_sum;
                        std::cout << "camera_pose x: " << camera_transform.transform.translation.x << std::endl;
                        std::cout << "Stair D: " << D << " m." << std::endl;
                        stair_size_csv << ros::Time::now() << "," << (int)trigger_msg.enable << "," << D << ",";
                        stair_count++;
                        need_new_H = true; // need to add new stair edge height
                    } else {
                        add_stair_edge = false; // no more stair edge to add
                    }//end if else
                }//end if

                if (need_new_H) {
                    if ( H_vector.size() >= stair_count) {
                        double next_edge_H = H_vector[stair_count-1];
                        H_sum += next_edge_H;
                        stair_climb.modify_last_edge_H(H_sum);
                        std::cout << "Stair H: " << next_edge_H << " m." << std::endl;
                        stair_size_csv << next_edge_H << "\n";
                        need_new_H = false;
                    }//end if
                }//end if

                eta_list = stair_climb.step();
                pitch = stair_climb.get_pitch();
                CoM = stair_climb.get_CoM();
                command_pitch_CoM << ros::Time::now() << "," << (int)trigger_msg.enable << "," << pitch << "," << CoM[0] << "," << CoM[1] << "\n";
                command_count ++;
                break;
            case UPPER_WALK:
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    walk_gait.initialize(current_eta, step_length);
                    walk_gait.set_step_height(0.04);
                }//end if
                pitch = 0.0;
                eta_list = walk_gait.step();
                command_count ++;
                break;
            default:
                break;
        }//end switch
        last_state = state;

        /* Next state */
        switch (state) {
            case INIT:
                state = TRANSFORM;
                break;
            case TRANSFORM:
                if (transform_ratio > 1.0) {
                    state = WAIT;
                }//end if
                break;
            case WAIT:
                if (trigger_msg.enable) {
                    state = WALK;
                }//end if
                break;
            case WALK:
                // Entering stair climbing phase
                touchdown_leg = walk_gait.get_touchdown_leg();
                if (touchdown_leg == 2 || touchdown_leg == 3) { // hind leg touched down (front leg start to swing)
                    if (hip_x + (max_step_length/2) >= to_stair_d - optimal_foothold) {   // max next foothold >= keep_stair_d_front_max, to swing up stair
                        state = STAIR;
                        std::cout << "Enter stair climbing phase." << std::endl;
                    }//end if
                }//end if
                break;
            case STAIR:
                if (!stair_climb.if_any_stair()) {
                    state = UPPER_WALK;
                }//end if  
                break;
            case UPPER_WALK:
                step_count = walk_gait.get_step_count();
                if (step_count[0] >= 2 && step_count[1] >= 2 && step_count[2] >= 2 && step_count[3] >= 2) { // all legs have stepped at least twice
                    state = END;
                }//end if  
                break;
            default:
                break;
        }//end switch

        /* Print when the leg contact stair edge */
        if_contact_edge = stair_climb.get_contact_edge_leg();
        for (int i=0; i<4; i++) {
            if (if_contact_edge[i] && !last_if_contact_edge[i]) {
                std::cout << "Command: " << command_count << ". Leg " << i << " is contacting the stair edge." << std::endl;
            }//end if
        }//end for
        last_if_contact_edge = if_contact_edge;

        /* Publish motor commands */
        for (int i=0; i<4; i++) {
            if (eta_list[0][i] > M_PI*160.0/180.0) {
                std::cout << "Leg " << i << " exceed upper bound." << std::endl;
                eta_list[0][i] = M_PI*160.0/180.0;
            }//end if 
            if (eta_list[0][i] < M_PI*16.9/180.0) {
                std::cout << "Leg " << i << " exceed lower bound." << std::endl;
                eta_list[0][i] = M_PI*16.9/180.0;
            }//end if 
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2)? (eta_list[1][i]-pitch) : -(eta_list[1][i]-pitch);
        }//end for
        motor_pub.publish(motor_cmd);
        auto one_loop_end = std::chrono::high_resolution_clock::now();
        auto one_loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(one_loop_end - one_loop_start);
        if (one_loop_duration.count() > max_cal_time) {
            max_cal_time = one_loop_duration.count();
            std::cout << "max time: " << max_cal_time << " us" << std::endl;
        }//end if
        rate.sleep();
    }//end while
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "max time: " << max_cal_time << " us" << std::endl;
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    std::cout << "total count: " << command_count << std::endl;

    stair_size_csv.close();
    command_pitch_CoM.close();
    ros::shutdown();
    return 0;
}//end main

