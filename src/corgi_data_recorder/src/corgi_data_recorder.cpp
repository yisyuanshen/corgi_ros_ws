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
#include "sensor_msgs/Imu.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/ForceStateStamped.h"
#include "corgi_msgs/SimDataStamped.h"
#include "geometry_msgs/Vector3.h"


bool trigger = false;
corgi_msgs::MotorCmdStamped motor_cmd;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::PowerCmdStamped power_cmd;
corgi_msgs::PowerStateStamped power_state;
sensor_msgs::Imu imu;
corgi_msgs::ImpedanceCmdStamped imp_cmd;
corgi_msgs::ForceStateStamped force_state;
corgi_msgs::SimDataStamped sim_data;
geometry_msgs::Vector3 odom_pos;
geometry_msgs::Vector3 odom_vel;

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
        output_file_path = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros_ws/output_data/" + output_file_name;

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
                        << "cmd_theta_a" << "," << "cmd_beta_a" << ","
                        << "cmd_trq_r_a" << "," << "cmd_trq_l_a" << ","
                        << "cmd_theta_b" << "," << "cmd_beta_b"  << ","
                        << "cmd_trq_r_b" << "," << "cmd_trq_l_b" << ","
                        << "cmd_theta_c" << "," << "cmd_beta_c"  << ","
                        << "cmd_trq_r_c" << "," << "cmd_trq_l_c" << ","
                        << "cmd_theta_d" << "," << "cmd_beta_d"  << ","
                        << "cmd_trq_r_d" << "," << "cmd_trq_l_d" << ","

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

                        << "imu_seq" << "," << "imu_sec" << "," << "imu_usec" << ","
                        << "imu_orien_x" << "," << "imu_orien_y" << "," << "imu_orien_z" << "," << "imu_orien_w" << ","
                        << "imu_ang_vel_x" << "," << "imu_ang_vel_y" << "," << "imu_ang_vel_z" << ","
                        << "imu_lin_acc_x" << "," << "imu_lin_acc_y" << "," << "imu_lin_acc_z" << ","

                        << "imp_seq" << "," << "imp_sec" << "," << "imp_usec" << ","
                        << "imp_cmd_theta_a" << "," << "imp_cmd_beta_a" << ","
                        << "imp_cmd_Fx_a"    << "," << "imp_cmd_Fy_a"   << ","
                        << "imp_cmd_theta_b" << "," << "imp_cmd_beta_b" << ","
                        << "imp_cmd_Fx_b"    << "," << "imp_cmd_Fy_b"   << ","
                        << "imp_cmd_theta_c" << "," << "imp_cmd_beta_c" << ","
                        << "imp_cmd_Fx_c"    << "," << "imp_cmd_Fy_c"   << ","
                        << "imp_cmd_theta_d" << "," << "imp_cmd_beta_d" << ","
                        << "imp_cmd_Fx_d"    << "," << "imp_cmd_Fy_d"   << ","

                        << "force_seq" << "," << "force_sec" << "," << "force_usec" << ","
                        << "force_Fx_a" << "," << "force_Fy_a" << ","
                        << "force_Fx_b" << "," << "force_Fy_b" << ","
                        << "force_Fx_c" << "," << "force_Fy_c" << ","
                        << "force_Fx_d" << "," << "force_Fy_d" << ","

                        << "sim_seq" << "," << "sim_sec" << "," << "sim_usec" << ","
                        << "sim_pos_x" << "," << "sim_pos_y" << "," << "sim_pos_z" << ","
                        << "sim_orien_x" << "," << "sim_orien_y" << "," << "sim_orien_z" << "," << "sim_orien_w" << ","

                        << "odom_pos_x" << "," << "odom_pos_y" << "," << "odom_pos_z" << ","
                        << "odom_vel_x" << "," << "odom_vel_y" << "," << "odom_vel_z" << ","

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

void imu_cb(const sensor_msgs::Imu values){
    imu = values;
}

void imp_cmd_cb(const corgi_msgs::ImpedanceCmdStamped cmd){
    imp_cmd = cmd;
}

void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}

void sim_data_cb(const corgi_msgs::SimDataStamped data){
    sim_data = data;
}

void odom_pos_cb(const geometry_msgs::Vector3::ConstPtr &msg){
    odom_pos = *msg;
}

void odom_vel_cb(const geometry_msgs::Vector3::ConstPtr &msg){
    odom_vel = *msg;
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

                << imu.header.seq << "," << imu.header.stamp.sec << "," << imu.header.stamp.nsec << ","
                << imu.orientation.x << "," << imu.orientation.y << "," << imu.orientation.z << "," << imu.orientation.w << ","
                << imu.angular_velocity.x << "," << imu.angular_velocity.y << "," << imu.angular_velocity.z << ","
                << imu.linear_acceleration.x << "," << imu.linear_acceleration.y << "," << imu.linear_acceleration.z << ","

                << imp_cmd.header.seq << "," << imp_cmd.header.stamp.sec << "," << imp_cmd.header.stamp.nsec << ","
                << imp_cmd.module_a.theta << "," << imp_cmd.module_a.beta << ","
                << imp_cmd.module_a.Fx    << "," << imp_cmd.module_a.Fy << ","
                << imp_cmd.module_b.theta << "," << imp_cmd.module_b.beta << ","
                << imp_cmd.module_b.Fx    << "," << imp_cmd.module_b.Fy << ","
                << imp_cmd.module_c.theta << "," << imp_cmd.module_c.beta << ","
                << imp_cmd.module_c.Fx    << "," << imp_cmd.module_c.Fy << ","
                << imp_cmd.module_d.theta << "," << imp_cmd.module_d.beta << ","
                << imp_cmd.module_d.Fx    << "," << imp_cmd.module_d.Fy << ","

                << force_state.header.seq << "," << force_state.header.stamp.sec << "," << force_state.header.stamp.nsec << ","
                << force_state.module_a.Fx    << "," << force_state.module_a.Fy << ","
                << force_state.module_b.Fx    << "," << force_state.module_b.Fy << ","
                << force_state.module_c.Fx    << "," << force_state.module_c.Fy << ","
                << force_state.module_d.Fx    << "," << force_state.module_d.Fy << ","

                << sim_data.header.seq << "," << sim_data.header.stamp.sec << "," << sim_data.header.stamp.nsec << ","
                << sim_data.position.x << "," << sim_data.position.y << "," << sim_data.position.z << ","
                << sim_data.orientation.x << "," << sim_data.orientation.y << "," << sim_data.orientation.z << "," << sim_data.orientation.w << ","

                << odom_pos.x << "," << odom_pos.y << "," << odom_pos.z << ","
                << odom_vel.x << "," << odom_vel.y << "," << odom_vel.z << ","

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
    
    ros::init(argc, argv, "corgi_data_recorder");

    ros::NodeHandle nh;
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Subscriber motor_cmd_sub = nh.subscribe<corgi_msgs::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber power_cmd_sub = nh.subscribe<corgi_msgs::PowerCmdStamped>("power/command", 1000, power_cmd_cb);
    ros::Subscriber power_state_sub = nh.subscribe<corgi_msgs::PowerStateStamped>("power/state", 1000, power_state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1000, imu_cb);
    ros::Subscriber imp_cmd_sub = nh.subscribe<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000, imp_cmd_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Subscriber sim_data_sub = nh.subscribe<corgi_msgs::SimDataStamped>("sim/data", 1000, sim_data_cb);
    ros::Subscriber odom_pos_sub = nh.subscribe<geometry_msgs::Vector3>("odometry/position", 1000, odom_pos_cb);
    ros::Subscriber odom_vel_sub = nh.subscribe<geometry_msgs::Vector3>("odometry/velocity", 1000, odom_vel_cb);
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
