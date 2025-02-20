#ifndef FSM_CONTROL_HPP
#define FSM_CONTROL_HPP

#include <iostream>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/FsmCmdStamped.h"
#include "corgi_msgs/FsmStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"

#define IDLE_MODE 0
#define CSV_MODE 1
#define WHEEL_MODE 2
#define WALK_MODE 3
#define WLW_MODE 4
#define STAIR_MODE 5

#endif