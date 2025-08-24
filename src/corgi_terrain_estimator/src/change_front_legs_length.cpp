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

    ROS_INFO("Change Front Legs Length Only\n");

    ros::init(argc, argv, "change_front_legs_length");

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
    
    // 前腿變化參數  
    double front_change_leg_length = 0.0;   // 前腿當前變化量
    double front_min_change = 0.0;          // 前腿最小變化量 (不變)
    double front_max_change = 0.03;         // 前腿最大變化量 (額外伸長3cm)
    double front_leg_increment = 0.0001;    // 前腿變化增量 (0.1mm)
    
    // 後腿保持不變
    double rear_change_leg_length = 0.0;    // 後腿變化量 (始終為0)

    double s = 0.0;

    ROS_INFO("Transform Starts - Setting initial stand height for all legs\n");
    for (int i=0; i<2000; i++){
        s = 0.0;
        
        // 前後腿都設定相同的初始站高
        eta_front = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {-s/1000.0, h/1000.0});
        eta_rear = legmodel.move(motor_cmd_modules[2]->theta, motor_cmd_modules[2]->beta, {-s/1000.0, h/1000.0});

        // 前腿設定 (module_a, module_b)
        motor_cmd_modules[0]->theta = eta_front[0];
        motor_cmd_modules[1]->theta = eta_front[0];
        motor_cmd_modules[0]->beta = -eta_front[1];
        motor_cmd_modules[1]->beta = eta_front[1];
        
        // 後腿設定 (module_c, module_d)
        motor_cmd_modules[2]->theta = eta_rear[0];
        motor_cmd_modules[3]->theta = eta_rear[0];
        motor_cmd_modules[2]->beta = eta_rear[1];
        motor_cmd_modules[3]->beta = -eta_rear[1];

        legmodel.contact_map(eta_front[0], eta_front[1]);

        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);

        rate.sleep();
    }
    ROS_INFO("Transform Ends\n");
        
    while (ros::ok()) {
        ros::spinOnce();
        
        if (trigger){
            ROS_INFO("Trigger received - Starting front legs extension sequence");
            int loop_count = 0;
            front_change_leg_length = front_min_change;  // 重置前腿變化為0
            
            while (ros::ok()) {
                double front_current_increment = 0.0;  // 前腿當前增量
                double rear_current_increment = 0.0;   // 後腿當前增量 (始終為0)

                if (loop_count < 500) {
                    front_current_increment = 0.0;  // 保持不變
                    ROS_INFO_THROTTLE(1.0, "Phase 1: Stabilizing at initial height");
                }
                else if (loop_count < 4000) {
                    if (loop_count < 1500) { 
                        // 前腿緩慢伸長
                        if (front_change_leg_length < front_max_change) {
                            front_current_increment = front_leg_increment;  // 前腿伸長
                            front_change_leg_length += front_leg_increment;
                            if (front_change_leg_length > front_max_change) {
                                front_change_leg_length = front_max_change;
                            }
                        }
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 2: Front legs extending, change: %.4f", front_change_leg_length);
                        }
                    } 
                    else if (loop_count < 2500) { 
                        front_current_increment = 0.0;  // 前腿保持最大伸長
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 3: Front legs holding at maximum: %.4f", front_change_leg_length);
                        }
                    } 
                    else if (loop_count < 4000) { 
                        // 前腿緩慢縮短回初始高度
                        if (front_change_leg_length > front_min_change) {
                            front_current_increment = -front_leg_increment;  // 前腿縮回
                            front_change_leg_length -= front_leg_increment;
                            if (front_change_leg_length < front_min_change) {
                                front_change_leg_length = front_min_change;
                            }
                        }
                        if (loop_count % 100 == 0) {
                            ROS_INFO("Phase 4: Front legs retracting, change: %.4f", front_change_leg_length);
                        }
                    }

                    // 前腿運動學計算 (基於初始站高 + 變化量)
                    eta_front = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {0.0, front_current_increment});
                    
                    // 後腿保持不變 (始終使用零增量)
                    eta_rear = legmodel.move(motor_cmd_modules[2]->theta, motor_cmd_modules[2]->beta, {0.0, rear_current_increment});

                    // 前腿角度限制檢查
                    double front_theta_deg = eta_front[0] * 180.0 / M_PI;
                    if (front_theta_deg > 160.0) {
                        eta_front[0] = 160.0 * M_PI / 180.0;
                        ROS_INFO("Front legs reached maximum theta degree: %.2f", front_theta_deg);
                    } else if (front_theta_deg < 17.0) {
                        eta_front[0] = 17.0 * M_PI / 180.0;
                        ROS_INFO("Front legs reached minimum theta degree: %.2f", front_theta_deg);
                    }

                    // 後腿角度限制檢查
                    double rear_theta_deg = eta_rear[0] * 180.0 / M_PI;
                    if (rear_theta_deg > 160.0) {
                        eta_rear[0] = 160.0 * M_PI / 180.0;
                    } else if (rear_theta_deg < 17.0) {
                        eta_rear[0] = 17.0 * M_PI / 180.0;
                    }

                    // 輸出狀態資訊
                    if (loop_count % 500 == 0) {
                        ROS_INFO("Loop: %d, Initial height: %.3f, Front change: %.4f, Front total: %.4f, Rear total: %.4f", 
                                loop_count, h, front_change_leg_length, h + front_change_leg_length, h + rear_change_leg_length);
                    }

                    // 前腿設定 (module_a = FL, module_b = FR)
                    motor_cmd_modules[0]->theta = eta_front[0];  // FL
                    motor_cmd_modules[1]->theta = eta_front[0];  // FR
                    motor_cmd_modules[0]->beta = -eta_front[1]; // FL
                    motor_cmd_modules[1]->beta = eta_front[1];  // FR

                    // 後腿設定 (module_c = RR, module_d = RL)
                    motor_cmd_modules[2]->theta = eta_rear[0];  // RR
                    motor_cmd_modules[3]->theta = eta_rear[0];  // RL
                    motor_cmd_modules[2]->beta = eta_rear[1];   // RR
                    motor_cmd_modules[3]->beta = -eta_rear[1];  // RL

                    legmodel.contact_map(eta_front[0], eta_front[1]);
                }
                else {
                    ROS_INFO("Front leg extension sequence completed");
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
