#include <iostream>

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"


int main(int argc, char **argv) {

    ROS_INFO("Impedance Command Publisher Starts\n");
    
    ros::init(argc, argv, "imp_cmd_pub");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Rate rate(1000);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };
    
    while (ros::ok()) {
        ros::spinOnce();
        
        for (auto& cmd : imp_cmd_modules){
            cmd->Px = 0;
            cmd->Py = -0.15;
            cmd->Fx = 0;
            cmd->Fy = 0;
            cmd->Mx = 0.652;
            cmd->My = 0.652;
            cmd->Kx = 30000;
            cmd->Ky = 30000;
            cmd->Dx = 400;
            cmd->Dy = 400;
            cmd->adaptive_kp_x = 30;
            cmd->adaptive_kp_y = 30;
            cmd->adaptive_ki_x = 10;
            cmd->adaptive_ki_y = 10;
            cmd->adaptive_kd_x = 20;
            cmd->adaptive_kd_y = 20;
        }

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}