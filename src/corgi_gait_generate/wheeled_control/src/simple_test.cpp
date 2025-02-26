#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include <std_msgs/String.h>

#include <corgi_msgs/SteeringStateStamped.h>
#include <corgi_msgs/SteeringCmdStamped.h>
#include <corgi_msgs/WheelCmd.h>
#include <corgi_msgs/MotorStateStamped.h>

class simple_test
{
private:
    void steeringStateCallback(const corgi_msgs::SteeringStateStamped::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Publisher  steering_cmd_pub_;
    ros::Publisher  wheel_cmd_pub_;
    ros::Publisher debug_pub_;
    ros::Subscriber steering_state_sub_;
    corgi_msgs::SteeringStateStamped current_steering_state_;
    corgi_msgs::MotorStateStamped current_motor_state_;
    int state;
    int add_wheel;
    void collaborate1();
    void collaborate2();
    void set_zero();
    
public:
    simple_test();
    ~simple_test() = default;
};

simple_test::simple_test()
{
    ros::NodeHandle pnh("~");
    // Publishers
    steering_cmd_pub_ = nh_.advertise<corgi_msgs::SteeringCmdStamped>("/steer/command", 1);
    wheel_cmd_pub_    = nh_.advertise<corgi_msgs::WheelCmd>("wheel_cmd", 1);
    debug_pub_ = nh_.advertise<std_msgs::String>("debug_info", 1);

    // Subscribers
    steering_state_sub_ = nh_.subscribe("/steer/state", 1,  &simple_test::steeringStateCallback, this);
    state =-1;
}


void simple_test::steeringStateCallback(const corgi_msgs::SteeringStateStamped::ConstPtr& msg)
{
  current_steering_state_ = *msg;
  if (current_steering_state_.cmd_finish == 1 )
  {
    std::cout << "Wrong cmd! "<< std::endl;
  }
  else if (current_steering_state_.current_state == true && state ==-1)
  {
    state = 0;
    collaborate1();
    std::cout << "sending1"<< std::endl;
  }
  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 0 &&state ==0)
  {
    collaborate1();
    std::cout << "sending1"<< std::endl;
  }

  else if(current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 2 &&state ==0)
  {
    state =1;
    std::cout << "finished1"<< std::endl;
  }

  else if (current_steering_state_.current_state == true && state ==1)
  {
    state =2;
    collaborate2();
    std::cout << "sending2"<< std::endl;
  }
  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 0 && state ==2 )
  {
    collaborate2();
    std::cout << "sending3"<< std::endl;
  }

  else if(current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 2 &&state ==2 && current_steering_state_.current_angle == -10)
  {
    state =3;
    std::cout << "finished4"<< std::endl;
  }


  else if (current_steering_state_.current_state == true && state ==3)
  {
    state =4;
    set_zero();
    std::cout << "sending5"<< std::endl;
  }

  else if (current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 0 && state ==4 )
  {
    set_zero();
    std::cout << "sending6"<< std::endl;
  }

  else if(current_steering_state_.current_state == true && current_steering_state_.cmd_finish == 2 &&state ==4)
  {
    state =5;
    std::cout << "finished7"<< std::endl;
  }
 
  
}

void simple_test::collaborate1()
{
    corgi_msgs::SteeringCmdStamped current_steering_cmd1;
    current_steering_cmd1.header.stamp = ros::Time::now();
    current_steering_cmd1.voltage = 4000;
    current_steering_cmd1.angle = 10;
    steering_cmd_pub_.publish(current_steering_cmd1);
    // std::cout << current_steering_cmd1<< std::endl;
}

void simple_test::collaborate2()
{
    corgi_msgs::SteeringCmdStamped current_steering_cmd2;
    current_steering_cmd2.header.stamp = ros::Time::now();
    current_steering_cmd2.voltage = 4000;
    current_steering_cmd2.angle = -10;
    steering_cmd_pub_.publish(current_steering_cmd2);
    // std::cout << current_steering_cmd2<< std::endl;
}

void simple_test::set_zero()
{
    corgi_msgs::SteeringCmdStamped current_steering_cmd3;
    current_steering_cmd3.header.stamp = ros::Time::now();
    current_steering_cmd3.voltage = 4000;
    current_steering_cmd3.angle = 0;
    steering_cmd_pub_.publish(current_steering_cmd3);
    // std::cout << current_steering_cmd3 << std::endl;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_test");
  simple_test test;
  ros::spin();
  return 0;
}