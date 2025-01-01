#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_odometry");

    ros::NodeHandle nh;
    ros::Rate rate(1000);

    
    while (ros::ok()){
        ros::spinOnce();
    }

    ros::shutdown();
    
    return 0;
}
