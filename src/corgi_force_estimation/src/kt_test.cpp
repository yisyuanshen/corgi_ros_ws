#include <iostream>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"

bool trigger = false;
corgi_msgs::MotorStateStamped motor_state;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped msg){
    motor_state = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_rt_control");

    ros::NodeHandle nh;
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);

    corgi_msgs::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    ROS_INFO("Leg Transform Starts\n");
    
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 150;
        cmd->kp_l = 150;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        cmd->kd_r = 1.75;
        cmd->kd_l = 1.75;
        cmd->torque_r = 0;
        cmd->torque_l = 0;
    }

    for (int i=0; i<2000; i++){
        motor_cmd.header.seq = -1;

        motor_cmd_pub.publish(motor_cmd);

        rate.sleep();
    }

    ROS_INFO("Leg Transform Finished\n");
    // int idx = 3;
    int seq = 0;
    while (ros::ok()){
        ros::spinOnce();

        if (!trigger) {
            rate.sleep();
            continue;
        }

        if (seq < 20000) {
            motor_cmd_modules[0]->beta -= 180/20000.0/180.0*M_PI;
            motor_cmd_modules[1]->beta += 180/20000.0/180.0*M_PI;
            motor_cmd_modules[2]->beta += 180/20000.0/180.0*M_PI;
            motor_cmd_modules[3]->beta -= 180/20000.0/180.0*M_PI;
        }
        else if (seq < 25000) {

        }
        else if (seq < 45000) {
            motor_cmd_modules[0]->beta += 180/20000.0/180.0*M_PI;
            motor_cmd_modules[1]->beta -= 180/20000.0/180.0*M_PI;
            motor_cmd_modules[2]->beta -= 180/20000.0/180.0*M_PI;
            motor_cmd_modules[3]->beta += 180/20000.0/180.0*M_PI;
        }
        else {
            break;
        }


        // if (std::abs(motor_state_modules[idx]->velocity_r) < 5) {
        //     if (seq%1000==0) {
        //         motor_cmd_modules[idx]->torque_r += 0.05;
        //     }
        // }
        // else {
        //     while (ros::ok()) {
        //         ros::spinOnce();
        //         if (motor_cmd_modules[idx]->torque_r > 0) {
        //                 motor_cmd_modules[idx]->torque_r -= 0.00001;
        //             }
        //         else {
        //             break;
        //         }
        //         motor_cmd_pub.publish(motor_cmd);
        //         rate.sleep();
        //     }
        //     break;
        // }

        // if (std::abs(motor_state_modules[idx]->velocity_l) < 5) {
        //     if (seq%1000==0) {
        //         motor_cmd_modules[idx]->torque_l += 0.05;
        //     }
        // }
        // else {
        //     motor_cmd_modules[idx]->torque_l *= 0.35;
        //     while (ros::ok()) {
        //         ros::spinOnce();
        //         if (motor_cmd_modules[idx]->torque_l > 0) {
        //             motor_cmd_modules[idx]->torque_l -= 0.000001;
        //         }
        //         else {
        //             break;
        //         }
        //         motor_cmd_pub.publish(motor_cmd);
        //         rate.sleep();
        //     }
        //     break;
        // }

        motor_cmd.header.seq = seq;

        motor_cmd_pub.publish(motor_cmd);

        seq++;

        rate.sleep();
    }

    ROS_INFO("Real Time Trajectory Finished\n");

    ros::shutdown();
    
    return 0;
}
