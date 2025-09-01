#include <iostream>

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "force_estimation.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {

    ROS_INFO("Impedance Command Publisher Starts (Vertical Motion)\n");
    
    ros::init(argc, argv, "imp_cmd_pub");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    // robot weight ~= 220 N
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Fx = 0;
        cmd->Fy = 0;
        cmd->Mx = 0;
        cmd->My = 0;
        if (sim) {
            cmd->Bx = 200;
            cmd->By = 200;
            cmd->Kx = 2000;
            cmd->Ky = 2000;
        }
        else {
            cmd->Bx = 80;
            cmd->By = 10;
            cmd->Kx = 2000;
            cmd->Ky = 100;
        }
    }

    std::array<double, 2> eta;
    double dh = 0.0;  // 改為高度變化量
    double mg = 19.68*9.81;

    // 設定高度範圍
    double h_min = 0.12;  // 最低高度
    double h_max = 0.18;  // 最高高度
    double h_current = h_min;  // 當前高度
    
    // 初始化到最低高度
    for (int i=0; i<2000; i++){
        eta = legmodel.move(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta, {0.0, (h_min - 0.12)/2000.0});
        
        imp_cmd_modules[0]->theta = eta[0];
        imp_cmd_modules[1]->theta = eta[0];
        imp_cmd_modules[2]->theta = eta[0];
        imp_cmd_modules[3]->theta = eta[0];

        imp_cmd_modules[0]->beta = -eta[1];
        imp_cmd_modules[1]->beta = eta[1];
        imp_cmd_modules[2]->beta = eta[1];
        imp_cmd_modules[3]->beta = -eta[1];
        
        // 所有腿平均承受重量（上下運動時重心不會偏移）
        imp_cmd_modules[0]->Fy = -mg/4.0;
        imp_cmd_modules[1]->Fy = -mg/4.0;
        imp_cmd_modules[2]->Fy = -mg/4.0;
        imp_cmd_modules[3]->Fy = -mg/4.0;

        imp_cmd.header.seq = -1;
        imp_cmd_pub.publish(imp_cmd);
        rate.sleep();
    }
        
    while (ros::ok()) {
        ros::spinOnce();
        
        if (trigger){
            int loop_count = 0;
            while (ros::ok()) {
                if (loop_count < 2000) {
                    dh = 0.0;  // 保持在最低點
                }
                else if (loop_count < 27000) {  // 增加到27000以容納兩個完整週期
                    // 創建上下擺動軌跡（兩個完整週期：0.12 -> 0.18 -> 0.12 -> 0.18 -> 0.12）
                    double cycle_length = 12000.0;  // 一個完整週期的長度
                    double phase = 2.0 * M_PI * (loop_count - 2000) / cycle_length;
                    
                    // 使用正弦波產生平滑的上下運動
                    double height_range = h_max - h_min;  // 0.06m
                    h_current = h_min + height_range * (1.0 + sin(phase)) / 2.0;
                    
                    // 計算相對於起始高度(0.12)的變化
                    dh = h_current - 0.12;
                    
                    eta = legmodel.move(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta, {0.0, dh/1000.0});

                    imp_cmd_modules[0]->theta = eta[0];
                    imp_cmd_modules[1]->theta = eta[0];
                    imp_cmd_modules[2]->theta = eta[0];
                    imp_cmd_modules[3]->theta = eta[0];

                    imp_cmd_modules[0]->beta = -eta[1];
                    imp_cmd_modules[1]->beta = eta[1];
                    imp_cmd_modules[2]->beta = eta[1];
                    imp_cmd_modules[3]->beta = -eta[1];

                    // 上下運動時，所有腿平均承受重量
                    imp_cmd_modules[0]->Fy = -mg/4.0;
                    imp_cmd_modules[1]->Fy = -mg/4.0;
                    imp_cmd_modules[2]->Fy = -mg/4.0;
                    imp_cmd_modules[3]->Fy = -mg/4.0;

                    // 可選：添加動態力擾動來測試系統響應
                    if (loop_count > 10000 && loop_count < 14000) {  // 在第二個週期中段添加擾動
                        double force_disturbance = 5.0 * sin((loop_count-10000)/200.0*M_PI);
                        imp_cmd_modules[0]->Fy += force_disturbance;
                        imp_cmd_modules[1]->Fy += force_disturbance;
                        imp_cmd_modules[2]->Fy += force_disturbance;
                        imp_cmd_modules[3]->Fy += force_disturbance;
                    }
                    
                    // 調試輸出 - 顯示週期資訊
                    if (loop_count % 500 == 0) {
                        double current_cycle = (double)(loop_count - 2000) / cycle_length;
                        ROS_INFO("Loop: %d, Cycle: %.2f, Target Height: %.3f, dh: %.3f", 
                                loop_count, current_cycle, h_current, dh);
                    }
                }
                else {
                    break;
                }

                imp_cmd.header.seq = loop_count;
                imp_cmd_pub.publish(imp_cmd);
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