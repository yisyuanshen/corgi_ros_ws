#include <iostream>
#include "ros/ros.h"

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"

bool sim = true;
bool trigger = false;

LegModel legmodel(sim);
std::array<double, 2> eta;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_rt_control");

    ros::NodeHandle nh;
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);

    corgi_msgs::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    ROS_INFO("Leg Transform Starts\n");
    
    for (auto& cmd : motor_cmd_modules) {
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 0.75;
            cmd->kd_l = 0.75;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    for (int i=0; i<2000; i++){
        motor_cmd_modules[0]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[1]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[2]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[3]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[0]->beta += 40/2000.0/180.0*M_PI;
        motor_cmd_modules[1]->beta -= 40/2000.0/180.0*M_PI;
        motor_cmd_modules[2]->beta -= 40/2000.0/180.0*M_PI;
        motor_cmd_modules[3]->beta += 40/2000.0/180.0*M_PI;

        motor_cmd.header.seq = -1;

        motor_cmd_pub.publish(motor_cmd);

        rate.sleep();
    }

    for (int i=0; i<1000; i++){
        rate.sleep();
    }

    ROS_INFO("Leg Transform Finished\n");

    while (ros::ok()){
        ros::spinOnce();

        if (trigger){
            ROS_INFO("Real Time Trajectory Starts\n");

            int loop_count = 0;
            while (ros::ok()) {
                if (loop_count < 1000) {
                }
                else if (loop_count < 3000) {
                    motor_cmd_modules[0]->theta -= 40/2000.0/180.0*M_PI;
                    motor_cmd_modules[1]->theta -= 40/2000.0/180.0*M_PI;
                    motor_cmd_modules[2]->theta -= 40/2000.0/180.0*M_PI;
                    motor_cmd_modules[3]->theta -= 40/2000.0/180.0*M_PI;
                }
                else if (loop_count < 5000) {
                    // move module A (single leg)
                    eta = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {0.04/1000.0, 0.04/1000.0});
                    motor_cmd_modules[0]->theta = eta[0];
                    motor_cmd_modules[0]->beta = eta[1];
                }
                else if (loop_count < 7000) {
                    // move module A (single leg)
                    eta = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {-0.08/1000.0, 0.0});
                    motor_cmd_modules[0]->theta = eta[0];
                    motor_cmd_modules[0]->beta = eta[1];
                }
                else if (loop_count < 9000) {
                    // move module A (single leg)
                    eta = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {0.04/1000.0, -0.04/1000.0});
                    motor_cmd_modules[0]->theta = eta[0];
                    motor_cmd_modules[0]->beta = eta[1];
                }
                else if (loop_count < 10000) {

                }
                else if (loop_count < 12000) {
                    // move module A (single leg)
                    motor_cmd_modules[0]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                    motor_cmd_modules[1]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                    motor_cmd_modules[2]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                    motor_cmd_modules[3]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                }
                else {
                    break;
                } 

                motor_cmd.header.seq = loop_count;

                motor_cmd_pub.publish(motor_cmd);

                loop_count++;

                rate.sleep();
            }
            break;
        }
    }

    ROS_INFO("Real Time Trajectory Finished\n");

    ros::shutdown();
    
    return 0;
}
