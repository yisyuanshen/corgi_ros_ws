#ifndef FORCE_ESTIMATE_HPP
#define FORCE_ESTIMATE_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_model.hpp"

#include "ros/ros.h"
#include "corgi_msgs/MotorStateStamped.h"


corgi_msgs::MotorStateStamped motor_state;

Eigen::MatrixXd H_l_poly(2, 8);
Eigen::MatrixXd U_l_poly(2, 8);
Eigen::MatrixXd F_l_poly(2, 8);
Eigen::MatrixXd L_l_poly(2, 8);
Eigen::MatrixXd G_poly(2, 8);
Eigen::MatrixXd L_r_poly(2, 8);
Eigen::MatrixXd F_r_poly(2, 8);
Eigen::MatrixXd U_r_poly(2, 8);
Eigen::MatrixXd P_poly(2, 8);

void motor_state_cb(const corgi_msgs::MotorStateStamped state);
Eigen::MatrixXd calculate_P_poly(int rim, double alpha);
Eigen::MatrixXd calculate_jacobian(Eigen::MatrixXd P_theta, Eigen::MatrixXd P_theta_deriv, double beta);
#endif