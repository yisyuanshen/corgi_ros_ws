#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <signal.h>

#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/PowerCmdStamped.h"
#include "corgi_msgs/PowerStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"

bool trigger = false;
corgi_msgs::MotorCmdStamped motor_cmd;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::PowerCmdStamped power_cmd;
corgi_msgs::PowerStateStamped power_state;

std::ofstream output_file;
std::string output_file_name = "";
std::string output_file_path = "";


bool file_exists(const std::string &filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}


void signal_handler(int signum) {
    ROS_INFO("Interrupt received.");

    if (output_file.is_open()) {
        output_file.close();
        ROS_INFO("Data successfully saved.");
    }

    ros::shutdown();
    exit(signum);
}


void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
    
    output_file_name = msg.output_filename;
    
    if (trigger && msg.output_filename != "") {
        output_file_path = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros_ws/src/corgi_control_packages/data_recorder/output_data/" + output_file_name;

        int index = 1;
        std::string file_path_with_extension = output_file_path + ".csv";
        while (file_exists(file_path_with_extension)) {
            file_path_with_extension = output_file_path + "_" + std::to_string(index) + ".csv";
            index++;
        }
        if (index != 1) output_file_name += "_" + std::to_string(index-1);
        output_file_name += ".csv";

        output_file_path = file_path_with_extension;

        if (!output_file.is_open()) {
            output_file.open(output_file_path);
            output_file << "Time" << ","
                        << "cmd_seq" << "," << "cmd_sec" << "," << "cmd_usec" << ","
                        << "cmd_theta_a"    << "," << "cmd_beta_a" << ","
                        << "cmd_trq_r_a" << "," << "cmd_trq_l_a" << ","
                        << "cmd_theta_b" << "," << "cmd_beta_b"  << ","
                        << "cmd_trq_r_b" << "," << "cmd_trq_l_b" << ","
                        << "cmd_theta_c" << "," << "cmd_beta_c"  << ","
                        << "cmd_trq_r_c" << "," << "cmd_trq_l_b" << ","
                        << "cmd_theta_d" << "," << "cmd_beta_d"  << ","
                        << "cmd_trq_r_d" << "," << "cmd_trq_l_b" << ","

                        << "state_seq" << "," << "state_sec" << "," << "state_usec" << ","
                        << "state_theta_a" << "," << "state_beta_a"  << ","
                        << "state_vel_r_a" << "," << "state_vel_l_a" << ","
                        << "state_trq_r_a" << "," << "state_trq_l_a" << ","
                        << "state_theta_b" << "," << "state_beta_b"  << ","
                        << "state_vel_r_b" << "," << "state_vel_l_b" << ","
                        << "state_trq_r_b" << "," << "state_trq_l_b" << ","
                        << "state_theta_c" << "," << "state_beta_c"  << ","
                        << "state_vel_r_c" << "," << "state_vel_l_c" << ","
                        << "state_trq_r_c" << "," << "state_trq_l_c" << ","
                        << "state_theta_d" << "," << "state_beta_d"  << ","
                        << "state_vel_r_d" << "," << "state_vel_l_d" << ","
                        << "state_trq_r_d" << "," << "state_trq_l_d" << ","

                        << "power_seq" << "," << "power_sec" << "," << "power_usec" << ","
                        << "v_0" << "," << "i_0" << ","
                        << "v_1" << "," << "i_1" << ","
                        << "v_2" << "," << "i_2" << ","
                        << "v_3" << "," << "i_3" << ","
                        << "v_4" << "," << "i_4" << ","
                        << "v_5" << "," << "i_5" << ","
                        << "v_6" << "," << "i_6" << ","
                        << "v_7" << "," << "i_7" << ","
                        << "v_8" << "," << "i_8" << ","
                        << "v_9" << "," << "i_9" << ","
                        << "v_10" << "," << "i_10" << ","
                        << "v_11" << "," << "i_11"
                        << "\n";

            ROS_INFO("Recording data to %s\n", output_file_name.c_str());
        }
    }
    else {
        if (output_file.is_open()) {
            output_file.close();
            ROS_INFO("Stopped recording data\n");
        }
    }
}


void motor_cmd_cb(const corgi_msgs::MotorCmdStamped cmd){
    motor_cmd = cmd;
}


void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}


void power_cmd_cb(const corgi_msgs::PowerCmdStamped cmd){
    power_cmd = cmd;
}


void power_state_cb(const corgi_msgs::PowerStateStamped state){
    power_state = state;
}


void write_data() {
    if (!output_file.is_open()){
        if (output_file_name != "") ROS_INFO("Output file is not opened\n");
        return;
    }

    output_file << ros::Time::now() << ","
                << motor_cmd.header.seq << "," << motor_cmd.header.stamp.sec << "," << motor_cmd.header.stamp.nsec << ","
                << motor_cmd.module_a.theta << "," << motor_cmd.module_a.beta << ","
                << motor_cmd.module_a.torque_r << "," << motor_cmd.module_a.torque_l << ","
                << motor_cmd.module_b.theta << "," << motor_cmd.module_b.beta << ","
                << motor_cmd.module_b.torque_r << "," << motor_cmd.module_b.torque_l << ","
                << motor_cmd.module_c.theta << "," << motor_cmd.module_c.beta << ","
                << motor_cmd.module_c.torque_r << "," << motor_cmd.module_c.torque_l << ","
                << motor_cmd.module_d.theta << "," << motor_cmd.module_d.beta << ","
                << motor_cmd.module_d.torque_r << "," << motor_cmd.module_d.torque_l << ","

                << motor_state.header.seq << "," << motor_state.header.stamp.sec << "," << motor_state.header.stamp.nsec << ","
                << motor_state.module_a.theta      << "," << motor_state.module_a.beta << ","
                << motor_state.module_a.velocity_r << "," << motor_state.module_a.velocity_l << ","
                << motor_state.module_a.torque_r   << "," << motor_state.module_a.torque_l << ","
                << motor_state.module_b.theta      << "," << motor_state.module_b.beta << ","
                << motor_state.module_b.velocity_r << "," << motor_state.module_b.velocity_l << ","
                << motor_state.module_b.torque_r   << "," << motor_state.module_b.torque_l << ","
                << motor_state.module_c.theta      << "," << motor_state.module_c.beta << ","
                << motor_state.module_c.velocity_r << "," << motor_state.module_c.velocity_l << ","
                << motor_state.module_c.torque_r   << "," << motor_state.module_c.torque_l << ","
                << motor_state.module_d.theta      << "," << motor_state.module_d.beta << ","
                << motor_state.module_d.velocity_r << "," << motor_state.module_d.velocity_l << ","
                << motor_state.module_d.torque_r   << "," << motor_state.module_d.torque_l << ","

                << power_state.header.seq << "," << power_state.header.stamp.sec << "," << power_state.header.stamp.nsec << ","
                << power_state.v_0 << "," << power_state.i_0 << ","
                << power_state.v_1 << "," << power_state.i_1 << ","
                << power_state.v_2 << "," << power_state.i_2 << ","
                << power_state.v_3 << "," << power_state.i_3 << ","
                << power_state.v_4 << "," << power_state.i_4 << ","
                << power_state.v_5 << "," << power_state.i_5 << ","
                << power_state.v_6 << "," << power_state.i_6 << ","
                << power_state.v_7 << "," << power_state.i_7 << ","
                << power_state.v_8 << "," << power_state.i_8 << ","
                << power_state.v_9 << "," << power_state.i_9 << ","
                << power_state.v_10 << "," << power_state.i_10 << ","
                << power_state.v_11 << "," << power_state.i_11
                << "\n";
                
    output_file.flush();
}


int main(int argc, char **argv) {
    ROS_INFO("Data Recorder Starts\n");
    
    ros::init(argc, argv, "data_recorder");

    ros::NodeHandle nh;
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Subscriber motor_cmd_sub = nh.subscribe<corgi_msgs::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber power_cmd_sub = nh.subscribe<corgi_msgs::PowerCmdStamped>("power/command", 1000, power_cmd_cb);
    ros::Subscriber power_state_sub = nh.subscribe<corgi_msgs::PowerStateStamped>("power/state", 1000, power_state_cb);
    ros::Rate rate(1000);

    signal(SIGINT, signal_handler);

    while (ros::ok()) {
        ros::spinOnce();

        if (trigger) {
            write_data();
        }

        rate.sleep();
    }

    if (output_file.is_open()) {
        output_file.close();
    }

    ros::shutdown();

    return 0;
}
