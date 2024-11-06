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
    ros::init(argc, argv, "csv_control");

    ros::NodeHandle nh;
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);

    corgi_msgs::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::MotorCmd*> motor_cmds = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    if (argc < 2){
        std::cerr << "Please input csv file path.\n";
        return 1;
    }
    
    std::string csv_file_path;
    csv_file_path = std::getenv("HOME");
    csv_file_path += "/corgi_ws/corgi_ros_ws/src/corgi_control_packages/csv_control/input_csv/";
    csv_file_path += argv[1];
    csv_file_path += ".csv";
    

    std::ifstream csv_file(csv_file_path);
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open the csv file.\n";
        return 1;
    }

    std::string line;
    
    while (ros::ok()){
        ros::spinOnce();

        if (trigger){
            ROS_INFO("Running the csv trajectory ...");

            int seq = 0;
            while (ros::ok() && std::getline(csv_file, line)) {
                std::vector<double> columns;
                std::stringstream ss(line);
                std::string item;
                
                for (auto& cmd : motor_cmds){
                    std::getline(ss, item, ',');
                    cmd->theta = std::stod(item);

                    std::getline(ss, item, ',');
                    cmd->beta = std::stod(item);

                    cmd->kp = 90;
                    cmd->ki = 0;
                    cmd->kd = 1.75;
                }

                motor_cmd.header.seq = seq;

                motor_cmd_pub.publish(motor_cmd);

                seq++;

                rate.sleep();
            }
            break;
        }
    }

    ROS_INFO("Shutting down the node...");

    ros::shutdown();
    
    return 0;
}