#ifndef WHEELED_HPP
#define WHEELED_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include <std_msgs/String.h>

#include "joystick_control.hpp"

#include <corgi_msgs/WheelCmd.h>
#include <corgi_msgs/MotorState.h>
#include <corgi_msgs/MotorStateStamped.h>
#include <corgi_msgs/MotorCmd.h>
#include <corgi_msgs/MotorCmdStamped.h>
#include <corgi_msgs/SteeringCmdStamped.h>

class Wheeled
{
public:
    Wheeled();
    ~Wheeled() = default;

private:
    void wheelCmdCallback(const corgi_msgs::WheelCmd::ConstPtr& msg);
    void motorsStateCallback(const corgi_msgs::MotorStateStamped::ConstPtr& msg);
    void steerStateCallback(const corgi_msgs::SteeringCmdStamped::ConstPtr& msg);

    
    ros::NodeHandle wnh_;
    ros::Publisher  motor_cmd_pub_;
    ros::Subscriber wheel_cmd_sub_;
    ros::Subscriber motor_state_sub_;
    ros::Subscriber steer_cmd_sub_;

    corgi_msgs::MotorStateStamped current_motor_state_;
    corgi_msgs::MotorCmdStamped current_motor_cmd_;
    corgi_msgs::WheelCmd current_wheel_cmd_;
    corgi_msgs::SteeringCmdStamped current_steer_cmd_;

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &current_motor_state_.module_a,
        &current_motor_state_.module_b,
        &current_motor_state_.module_c,
        &current_motor_state_.module_d
    };

    std::vector<corgi_msgs::MotorCmd*> motor_cmds = {
        &current_motor_cmd_.module_a,
        &current_motor_cmd_.module_b,
        &current_motor_cmd_.module_c,
        &current_motor_cmd_.module_d
    };
};


#endif