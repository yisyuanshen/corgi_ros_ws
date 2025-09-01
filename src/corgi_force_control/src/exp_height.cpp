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
    double dh = 0.0;
    double mg = 19.68*9.81;

    double h = 0.12;
    double h_range = 0.06; // 0.18 - 0.12 = 0.06
    
    // 初始化階段：移動到起始高度
    for (int i=0; i<2000; i++){
        eta = legmodel.move(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta, {0.0, h/2000.0});
        
        imp_cmd_modules[0]->theta = eta[0];
        imp_cmd_modules[1]->theta = eta[0];
        imp_cmd_modules[2]->theta = eta[0];
        imp_cmd_modules[3]->theta = eta[0];

        imp_cmd_modules[0]->beta = -eta[1];
        imp_cmd_modules[1]->beta = eta[1];
        imp_cmd_modules[2]->beta = eta[1];
        imp_cmd_modules[3]->beta = -eta[1];
        
        // 四條腿平均承受重量
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
                    dh = 0.0;
                }
                else if (loop_count < 18000) {
                    // 第一個週期 (2000-10000)
                    if (loop_count < 2200) { 
                        dh += 2*h_range/2000.0/200.0;  // 加速上升
                    }
                    else if (loop_count < 3800) { 
                        dh = 2*h_range/2000.0;  // 等速上升
                    }
                    else if (loop_count < 4000) { 
                        dh -= 2*h_range/2000.0/200.0;  // 減速到頂點
                    }
                    else if (loop_count < 4200) { 
                        dh -= 2*h_range/2000.0/200.0;  // 加速下降
                    }
                    else if (loop_count < 5800) { 
                        dh = -2*h_range/2000.0;  // 等速下降
                    }
                    else if (loop_count < 6000) { 
                        dh += 2*h_range/2000.0/200.0;  // 減速到底點
                    }
                    else if (loop_count < 6200) { 
                        dh += 2*h_range/2000.0/200.0;  // 加速上升
                    }
                    else if (loop_count < 7800) { 
                        dh = 2*h_range/2000.0;  // 等速上升
                    }
                    else if (loop_count < 8000) { 
                        dh -= 2*h_range/2000.0/200.0;  // 減速到頂點
                    }
                    else if (loop_count < 8200) { 
                        dh -= 2*h_range/2000.0/200.0;  // 加速下降
                    }
                    else if (loop_count < 9800) { 
                        dh = -2*h_range/2000.0;  // 等速下降
                    }
                    else if (loop_count < 10000) { 
                        dh += 2*h_range/2000.0/200.0;  // 減速到底點
                    }
                    // 第二個週期 (10000-18000)
                    else if (loop_count < 10200) { 
                        dh += 2*h_range/2000.0/200.0;  // 加速上升
                    }
                    else if (loop_count < 11800) { 
                        dh = 2*h_range/2000.0;  // 等速上升
                    }
                    else if (loop_count < 12000) { 
                        dh -= 2*h_range/2000.0/200.0;  // 減速到頂點
                    }
                    else if (loop_count < 12200) { 
                        dh -= 2*h_range/2000.0/200.0;  // 加速下降
                    }
                    else if (loop_count < 13800) { 
                        dh = -2*h_range/2000.0;  // 等速下降
                    }
                    else if (loop_count < 14000) { 
                        dh += 2*h_range/2000.0/200.0;  // 減速到底點
                    }
                    else if (loop_count < 14200) { 
                        dh += 2*h_range/2000.0/200.0;  // 加速上升
                    }
                    else if (loop_count < 15800) { 
                        dh = 2*h_range/2000.0;  // 等速上升
                    }
                    else if (loop_count < 16000) { 
                        dh -= 2*h_range/2000.0/200.0;  // 減速到頂點
                    }
                    else if (loop_count < 16200) { 
                        dh -= 2*h_range/2000.0/200.0;  // 加速下降
                    }
                    else if (loop_count < 17800) { 
                        dh = -2*h_range/2000.0;  // 等速下降
                    }
                    else if (loop_count < 18000) { 
                        dh += 2*h_range/2000.0/200.0;  // 減速回到起始點
                    }

                    eta = legmodel.move(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta, {0.0, dh});

                    imp_cmd_modules[0]->theta = eta[0];
                    imp_cmd_modules[1]->theta = eta[0];
                    imp_cmd_modules[2]->theta = eta[0];
                    imp_cmd_modules[3]->theta = eta[0];

                    imp_cmd_modules[0]->beta = -eta[1];
                    imp_cmd_modules[1]->beta = eta[1];
                    imp_cmd_modules[2]->beta = eta[1];
                    imp_cmd_modules[3]->beta = -eta[1];

                    // 四條腿平均承受重量（上下運動時重心不偏移）
                    imp_cmd_modules[0]->Fy = -mg/4.0;
                    imp_cmd_modules[1]->Fy = -mg/4.0;
                    imp_cmd_modules[2]->Fy = -mg/4.0;
                    imp_cmd_modules[3]->Fy = -mg/4.0;

                    // 在第二個週期添加力擾動測試（模仿原程式）
                    if (loop_count > 14000 && loop_count < 18000) {
                        imp_cmd_modules[0]->Fy += 10 * sin((loop_count-2000)/500.0*M_PI);
                        imp_cmd_modules[1]->Fy -= 10 * sin((loop_count-2000)/500.0*M_PI);
                        imp_cmd_modules[2]->Fy += 10 * sin((loop_count-2000)/500.0*M_PI);
                        imp_cmd_modules[3]->Fy -= 10 * sin((loop_count-2000)/500.0*M_PI);
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