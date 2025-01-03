#ifndef IMPEDANCE_CONTROL_HPP
#define IMPEDANCE_CONTROL_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_model.hpp"

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/ForceStateStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/MotorCmdStamped.h"

corgi_msgs::ImpedanceCmdStamped imp_cmd;
corgi_msgs::ForceStateStamped force_state;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::MotorCmdStamped motor_cmd;




#endif