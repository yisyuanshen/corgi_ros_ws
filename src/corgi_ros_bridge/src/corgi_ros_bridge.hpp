#include "corgi_ros_bridge/LegStamped.h"
#include "corgi_ros_bridge/RobotStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <mutex>
#include <ros/ros.h>
#include <sys/time.h>
#include <rosgraph_msgs/Clock.h>

#include "force.pb.h"
#include "motor.pb.h"
#include "power.pb.h"
#include "robot.pb.h"
#include <NodeHandler.h>


motor_msg::MotorStamped motor_fb_msg;
force_msg::LegForceStamped force_fb_msg;
robot_msg::StateStamped robot_fb_msg;

corgi_ros_bridge::RobotStamped ros_robot_fb_msg;
corgi_ros_bridge::RobotStamped robot_state_msg;

int motor_msg_updated;
int force_msg_updated;
int robot_msg_updated;
int ros_robot_msg_updated;

std::mutex mtx;

std::vector<double> KP;
std::vector<double> KI;
std::vector<double> KD;
std::vector<std::vector<double>> M_d;
std::vector<std::vector<double>> K_0;
std::vector<std::vector<double>> D_d;
std::vector<std::vector<double>> adaptive_pid_x;
std::vector<std::vector<double>> adaptive_pid_y;


void motor_feedback_cb(motor_msg::MotorStamped msg);
void force_feedback_cb(force_msg::LegForceStamped msg);
void robot_feedback_cb(robot_msg::State msg);
void ros_robot_feedback_cb(corgi_ros_bridge::RobotStamped msg);
void load_config();
