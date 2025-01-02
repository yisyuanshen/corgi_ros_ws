#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/ForceStateStamped.h"

corgi_msgs::MotorCmdStamped motor_cmd;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::ForceStateStamped force_state;

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}

int main(int argc, char **argv) {

    ROS_INFO("Impedance Control Test Starts\n");

    ros::init(argc, argv, "impedance_control_test");

    ros::NodeHandle nh;
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Rate rate(1000);

    std::vector<corgi_msgs::MotorCmd*> motor_cmds = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    for (auto & cmd : motor_cmds){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0;
        cmd->kp = 90;
        cmd->ki = 0;
        cmd->kd = 17.5;
    }

    int seq = 0;
    while (ros::ok()) {
        ros::spinOnce();

        motor_cmd.module_a.theta = 17/180.0*M_PI + 1 - cos(seq/5000.0*M_PI);
        motor_cmd.module_b.theta = 17/180.0*M_PI + 1 - cos(seq/5000.0*M_PI);
        motor_cmd.module_c.theta = 17/180.0*M_PI + 1 - cos(seq/5000.0*M_PI);
        motor_cmd.module_d.theta = 17/180.0*M_PI + 1 - cos(seq/5000.0*M_PI);

        motor_cmd.module_a.beta = 1.5 * -sin(seq/10000.0*M_PI);
        motor_cmd.module_b.beta = 1.5 * sin(seq/10000.0*M_PI);
        motor_cmd.module_c.beta = 1.5 * sin(seq/10000.0*M_PI);
        motor_cmd.module_d.beta = 1.5 * -sin(seq/10000.0*M_PI);

        std::cout << "seq = " << force_state.header.seq << std::endl
                  << force_state.module_a.fx << ", " << force_state.module_a.fz << std::endl
                  << force_state.module_b.fx << ", " << force_state.module_b.fz << std::endl
                  << force_state.module_c.fx << ", " << force_state.module_c.fz << std::endl
                  << force_state.module_d.fx << ", " << force_state.module_d.fz << std::endl
                  << std::endl;
        

        motor_cmd_pub.publish(motor_cmd);

        seq++;

        rate.sleep();
    }

    ros::shutdown();



    return 0;
}