#include <iostream>

#include "ros/ros.h"
#include "corgi_msgs/MotorStateStamped.h"


int main(int argc, char **argv) {

    ROS_INFO("Motor State Publisher Starts\n");
    
    ros::init(argc, argv, "motor_state_pub");

    ros::NodeHandle nh;
    ros::Publisher motor_state_pub = nh.advertise<corgi_msgs::MotorStateStamped>("motor/state", 1000);
    ros::Rate rate(1000);

    corgi_msgs::MotorStateStamped motor_state;

    std::vector<corgi_msgs::MotorState*> motor_states = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };
    
    while (ros::ok()) {
        ros::spinOnce();
        
        for (auto& state : motor_states){
            state->theta = 17/180.0*M_PI;
            state->beta = 0;
            state->velocity_r = 0;
            state->velocity_l = 0;
            state->torque_r = -2.265;
            state->torque_l = 2.265;
        }

        motor_state.header.seq = -1;

        motor_state_pub.publish(motor_state);

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}