#include <iostream>
#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "force_estimation.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {

    ROS_INFO("Change Leg Length in the same height\n");

    ros::init(argc, argv, "change_leg_length_same_height");

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

    // initialize motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 1;
            cmd->kd_l = 1;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    std::array<double, 2> eta;
    double mg = 19.68*9.81;
    
    // 初始站高設定 (固定值)
    double h = 0.03;                        // 初始站高 15cm
    
    // 腿長變化參數  
    double change_leg_length = 0.0;         // 當前腿長變化量
    double min_change = 0.0;                // 最小變化量 (不變)
    double max_change = 0.03;               // 最大變化量 (額外伸長3cm)
    double leg_increment = 0.0001;          // 腿長變化增量 (0.1mm)

    double s = 0.0;

    ROS_INFO("Transform Starts - Setting initial stand height\n");
    for (int i=0; i<2000; i++){
        s = 0.0;
        // 設定固定的初始站高，腿長變化為0
        eta = legmodel.move(motor_cmd_modules[1]->theta, motor_cmd_modules[1]->beta, {-s/1000.0, h/1000.0});

        motor_cmd_modules[0]->theta = eta[0];
        motor_cmd_modules[1]->theta = eta[0];
        motor_cmd_modules[2]->theta = eta[0];
        motor_cmd_modules[3]->theta = eta[0];

        motor_cmd_modules[0]->beta = -eta[1];
        motor_cmd_modules[1]->beta = eta[1];
        motor_cmd_modules[2]->beta = eta[1];
        motor_cmd_modules[3]->beta = -eta[1];

        legmodel.contact_map(eta[0], eta[1]);

        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);

        rate.sleep();
    }
    ROS_INFO("Transform Ends\n");
        
    while (ros::ok()) {
        ros::spinOnce();
        
        if (trigger){
            ROS_INFO("Trigger received - Starting leg length change sequence");
            int loop_count = 0;
            change_leg_length = min_change;  // 重置腿長變化為0
            
            while (ros::ok()) {
                double current_leg_increment = 0.0;  // 當前迴圈的腿長變化增量

                if (loop_count < 500) {
                    current_leg_increment = 0.0;  // 保持不變
                    ROS_INFO_THROTTLE(1.0, "Phase 1: Stabilizing at initial height");
                }
                else if (loop_count < 4000) {
                    if (loop_count < 1500) { 
                        // 緩慢伸長腿部
                        if (change_leg_length < max_change) {
                            current_leg_increment = leg_increment;  // 伸長
                            change_leg_length += leg_increment;
                            if (change_leg_length > max_change) {
                                change_leg_length = max_change;
                            }
                        }
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 2: Extending legs, change: %.4f", change_leg_length);
                        }
                    } 
                    else if (loop_count < 2500) { 
                        current_leg_increment = 0.0;  // 保持最大伸長
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 3: Holding at maximum extension: %.4f", change_leg_length);
                        }
                    } 
                    else if (loop_count < 4000) { 
                        // 緩慢縮短腿部
                        if (change_leg_length > min_change) {
                            current_leg_increment = -leg_increment;  // 縮回
                            change_leg_length -= leg_increment;
                            if (change_leg_length < min_change) {
                                change_leg_length = min_change;
                            }
                        }
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 4: Retracting legs, change: %.4f", change_leg_length);
                        }
                    }

                    // 運動學計算：基於固定初始站高 + 腿長變化
                    eta = legmodel.move(motor_cmd_modules[1]->theta, motor_cmd_modules[1]->beta, {0.0, current_leg_increment});

                    // 角度限制檢查
                    double theta_deg = eta[0] * 180.0 / M_PI;
                    if (theta_deg > 160.0) {
                        eta[0] = 160.0 * M_PI / 180.0;  // 限制在最大值
                        ROS_INFO("Reached maximum theta degree: %.2f", theta_deg);
                    } else if (theta_deg < 17.0) {
                        eta[0] = 17.0 * M_PI / 180.0;   // 限制在最小值
                        ROS_INFO("Reached minimum theta degree: %.2f", theta_deg);
                    }

                    // 輸出狀態資訊
                    if (loop_count % 500 == 0) {
                        ROS_INFO("Loop: %d, Initial height: %.3f, Leg change: %.4f, Total height: %.4f, Theta: %.2f deg", 
                                loop_count, h, change_leg_length, h + change_leg_length, theta_deg);
                    }

                    motor_cmd_modules[0]->theta = eta[0];
                    motor_cmd_modules[1]->theta = eta[0];
                    motor_cmd_modules[2]->theta = eta[0];
                    motor_cmd_modules[3]->theta = eta[0];

                    motor_cmd_modules[0]->beta = -eta[1];
                    motor_cmd_modules[1]->beta = eta[1];
                    motor_cmd_modules[2]->beta = eta[1];
                    motor_cmd_modules[3]->beta = -eta[1];

                    legmodel.contact_map(eta[0], eta[1]);
                }
                else {
                    break;
                }
                motor_cmd.header.seq = loop_count;
                motor_cmd_pub.publish(motor_cmd);

                loop_count++;

                rate.sleep();
            }
            ros::shutdown();
            return 0;
        }
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}