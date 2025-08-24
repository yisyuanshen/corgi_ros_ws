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

    ROS_INFO("Change Rear Legs Length Only\n");

    ros::init(argc, argv, "change_rear_legs_length");

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

    std::array<double, 2> eta_front; 
    std::array<double, 2> eta_rear; 
    
    double mg = 19.68*9.81;
    
    // 初始站高設定 (固定值)
    double h = 0.03;                        // 初始站高 3cm
    
    // 後腿長變化參數  
    double rear_change_leg_length = 0.0;    // 當前後腿長變化量
    double min_change = 0.0;                // 最小變化量 (不變)
    double max_change = 0.03;               // 最大變化量 (額外伸長3cm)
    double leg_increment = 0.0001;          // 腿長變化增量 (0.1mm)            

    double s = 0.0;

    ROS_INFO("Transform Starts - Setting initial stand height\n");
    for (int i=0; i<2000; i++){
        s = 0.0;
        // 設定固定的初始站高，腿長變化為0
        eta_front = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {-s/1000.0, h/1000.0});
        eta_rear = legmodel.move(motor_cmd_modules[2]->theta, motor_cmd_modules[2]->beta, {-s/1000.0, h/1000.0});

        // 前腿設定 (module_a, module_b)
        motor_cmd_modules[0]->theta = eta_front[0];  // FL
        motor_cmd_modules[1]->theta = eta_front[0];  // FR
        motor_cmd_modules[0]->beta = -eta_front[1];  // FL
        motor_cmd_modules[1]->beta = eta_front[1];   // FR
        
        // 後腿設定 (module_c, module_d)
        motor_cmd_modules[2]->theta = eta_rear[0];   // RR
        motor_cmd_modules[3]->theta = eta_rear[0];   // RL
        motor_cmd_modules[2]->beta = eta_rear[1];    // RR
        motor_cmd_modules[3]->beta = -eta_rear[1];   // RL

        legmodel.contact_map(eta_front[0], eta_front[1]);

        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);

        rate.sleep();
    }
    ROS_INFO("Transform Ends\n");
        
    while (ros::ok()) {
        ros::spinOnce();
        
        if (trigger){
            ROS_INFO("Trigger received - Starting rear leg length change sequence");
            int loop_count = 0;
            rear_change_leg_length = min_change;  // 重置後腿長變化為0
            
            while (ros::ok()) {
                double rear_current_increment = 0.0;  // 當前迴圈的後腿長變化增量

                if (loop_count < 500) {
                    rear_current_increment = 0.0;  // 保持不變
                    ROS_INFO_THROTTLE(1.0, "Phase 1: Stabilizing at initial height");
                }
                else if (loop_count < 4000) {
                    if (loop_count < 1500) { 
                        // 緩慢伸長後腿
                        if (rear_change_leg_length < max_change) {
                            rear_current_increment = leg_increment;  // 伸長
                            rear_change_leg_length += leg_increment;
                            if (rear_change_leg_length > max_change) {
                                rear_change_leg_length = max_change;
                            }
                        }
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 2: Extending rear legs, change: %.4f", rear_change_leg_length);
                        }
                    } 
                    else if (loop_count < 2500) { 
                        rear_current_increment = 0.0;  // 保持最大伸長
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 3: Holding rear legs at maximum extension: %.4f", rear_change_leg_length);
                        }
                    } 
                    else if (loop_count < 4000) { 
                        // 緩慢縮短後腿
                        if (rear_change_leg_length > min_change) {
                            rear_current_increment = -leg_increment;  // 縮回
                            rear_change_leg_length -= leg_increment;
                            if (rear_change_leg_length < min_change) {
                                rear_change_leg_length = min_change;
                            }
                        }
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 4: Retracting rear legs, change: %.4f", rear_change_leg_length);
                        }
                    }

                    // 前腿運動學計算 - 保持不變
                    eta_front = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {0.0, 0.0});
                    
                    // 後腿運動學計算 - 使用增量變化
                    eta_rear = legmodel.move(motor_cmd_modules[2]->theta, motor_cmd_modules[2]->beta, {0.0, rear_current_increment});

                    // 前腿角度限制檢查
                    double front_theta_deg = eta_front[0] * 180.0 / M_PI;
                    if (front_theta_deg > 160.0) {
                        eta_front[0] = 160.0 * M_PI / 180.0;  // 限制在最大值
                    } else if (front_theta_deg < 17.0) {
                        eta_front[0] = 17.0 * M_PI / 180.0;   // 限制在最小值
                    }

                    // 後腿角度限制檢查
                    double rear_theta_deg = eta_rear[0] * 180.0 / M_PI;
                    if (rear_theta_deg > 160.0) {
                        eta_rear[0] = 160.0 * M_PI / 180.0;
                        ROS_INFO("Rear legs reached maximum theta degree: %.2f", rear_theta_deg);
                    } else if (rear_theta_deg < 17.0) {
                        eta_rear[0] = 17.0 * M_PI / 180.0;
                        ROS_INFO("Rear legs reached minimum theta degree: %.2f", rear_theta_deg);
                    }

                    // 輸出狀態資訊
                    if (loop_count % 500 == 0) {
                        ROS_INFO("Loop: %d, Initial height: %.3f, Rear leg change: %.4f, Rear total height: %.4f, Front theta: %.2f deg, Rear theta: %.2f deg", 
                                loop_count, h, rear_change_leg_length, h + rear_change_leg_length, front_theta_deg, rear_theta_deg);
                    }

                    // 前腿設定 (module_a = FL, module_b = FR) - 保持不變
                    motor_cmd_modules[0]->theta = eta_front[0];  // FL
                    motor_cmd_modules[1]->theta = eta_front[0];  // FR
                    motor_cmd_modules[0]->beta = -eta_front[1]; // FL
                    motor_cmd_modules[1]->beta = eta_front[1];  // FR

                    // 後腿設定 (module_c = RR, module_d = RL) - 使用變化後的值
                    motor_cmd_modules[2]->theta = eta_rear[0];  // RR
                    motor_cmd_modules[3]->theta = eta_rear[0];  // RL
                    motor_cmd_modules[2]->beta = eta_rear[1];   // RR
                    motor_cmd_modules[3]->beta = -eta_rear[1];  // RL

                    legmodel.contact_map(eta_front[0], eta_front[1]);
                }
                else {
                    ROS_INFO("Movement sequence completed");
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
