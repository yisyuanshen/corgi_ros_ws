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

void motor_cmd_init(){
    motor_cmd.module_a.theta = 17/180.0*M_PI;
    motor_cmd.module_b.theta = 17/180.0*M_PI;
    motor_cmd.module_c.theta = 17/180.0*M_PI;
    motor_cmd.module_d.theta = 17/180.0*M_PI;

    motor_cmd.module_a.kp = 90;
    motor_cmd.module_b.kp = 90;
    motor_cmd.module_c.kp = 90;
    motor_cmd.module_d.kp = 90;

    motor_cmd.module_a.kd = 1.75;
    motor_cmd.module_b.kd = 1.75;
    motor_cmd.module_c.kd = 1.75;
    motor_cmd.module_d.kd = 1.75;
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

        // motor_cmd.module_a.beta = -sin(seq/10000.0*M_PI);
        // motor_cmd.module_b.beta = sin(seq/10000.0*M_PI);
        // motor_cmd.module_c.beta = sin(seq/10000.0*M_PI);
        // motor_cmd.module_d.beta = -sin(seq/10000.0*M_PI);


        motor_cmd_pub.publish(motor_cmd);

        seq++;

        rate.sleep();
    }

    ros::shutdown();



    return 0;
}