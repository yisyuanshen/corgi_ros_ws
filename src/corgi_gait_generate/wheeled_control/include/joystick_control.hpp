#ifndef JOYSTICK_CONTROL_HPP
#define JOYSTICK_CONTROL_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>
#include <string>

// Custom messages
#include <corgi_msgs/SteeringStateStamped.h>
#include <corgi_msgs/SteeringCmdStamped.h>
#include <corgi_msgs/WheelCmd.h>

class JoystickControl
{
public:
  JoystickControl();

private:
  // Callbacks
  void steeringStateCallback(const corgi_msgs::SteeringStateStamped::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  // Timer callback for 1 kHz
  void wheelCmdTimerCallback(const ros::TimerEvent&);
  void steerCmdTimerCallback(const ros::TimerEvent&);

  double clamp(double value, double min_val, double max_val);

  // ROS members
  ros::NodeHandle nh_;
  ros::Publisher steering_cmd_pub_;
  ros::Publisher wheel_cmd_pub_;
  ros::Publisher debug_pub_;

  ros::Subscriber steering_state_sub_;
  ros::Subscriber joy_sub_;

  // Timer at 1 kHz
  ros::Timer wheel_cmd_timer_;
  ros::Timer steering_cmd_timer_;
  // Store the current steering state
  corgi_msgs::SteeringStateStamped current_steering_state_;
  corgi_msgs::SteeringCmdStamped steer;
  // Indices for axes/buttons
  int axis_left_right_;
  int axis_forward_back_;
  int axis_velocity_;
  int button_hold_;
  int button_reset_;

  // Logic
  bool hold_active_;
  bool was_hold_pressed_;
  bool was_reset_pressed_;

  bool last_direction_;   
  double current_velocity_;

  // The last WheelCmd we published
  corgi_msgs::WheelCmd last_wheel_cmd_;
};

#endif // JOYSTICK_CONTROL_HPP
