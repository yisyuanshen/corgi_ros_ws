#include <iostream>
#include <mutex>
#include "ros/ros.h"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"

#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/PowerCmdStamped.h"
#include "corgi_msgs/PowerStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"


std::mutex mutex_ros_motor_state;
std::mutex mutex_ros_power_state;
std::mutex mutex_grpc_motor_cmd;
std::mutex mutex_grpc_power_cmd;

corgi_msgs::MotorCmdStamped     ros_motor_cmd;
corgi_msgs::PowerCmdStamped     ros_power_cmd;
corgi_msgs::MotorStateStamped   ros_motor_state;
corgi_msgs::PowerStateStamped   ros_power_state;

motor_msg::MotorCmdStamped      grpc_motor_cmd;
power_msg::PowerCmdStamped      grpc_power_cmd;
motor_msg::MotorStateStamped    grpc_motor_state;
power_msg::PowerStateStamped    grpc_power_state;

ros::Publisher ros_motor_state_pub;
ros::Publisher ros_power_state_pub;
core::Publisher<motor_msg::MotorCmdStamped> *grpc_motor_cmd_pub;
core::Publisher<power_msg::PowerCmdStamped> *grpc_power_cmd_pub;

bool ros_trigger = false;


void ros_trigger_cb(const corgi_msgs::TriggerStamped trigger){
    ros_trigger = trigger.enable;
}

void ros_motor_cmd_cb(const corgi_msgs::MotorCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);

    ros_motor_cmd = cmd;

    std::vector<motor_msg::MotorCmd*> grpc_motor_modules = {
        grpc_motor_cmd.mutable_module_a(),
        grpc_motor_cmd.mutable_module_b(),
        grpc_motor_cmd.mutable_module_c(),
        grpc_motor_cmd.mutable_module_d()
    };

    std::vector<corgi_msgs::MotorCmd> ros_motor_modules = {
        ros_motor_cmd.module_a,
        ros_motor_cmd.module_b,
        ros_motor_cmd.module_c,
        ros_motor_cmd.module_d
    };

    for (int i = 0; i < 4; i++) {
        grpc_motor_modules[i]->set_theta(ros_motor_modules[i].theta);
        grpc_motor_modules[i]->set_beta(ros_motor_modules[i].beta);
        grpc_motor_modules[i]->set_kp(ros_motor_modules[i].kp);
        grpc_motor_modules[i]->set_ki(ros_motor_modules[i].ki);
        grpc_motor_modules[i]->set_kd(ros_motor_modules[i].kd);
        grpc_motor_modules[i]->set_torque_r(ros_motor_modules[i].torque_r);
        grpc_motor_modules[i]->set_torque_l(ros_motor_modules[i].torque_l);
    }

    grpc_motor_cmd.mutable_header()->set_seq(ros_motor_cmd.header.seq);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(ros_motor_cmd.header.stamp.sec);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(ros_motor_cmd.header.stamp.nsec);

    grpc_motor_cmd_pub->publish(grpc_motor_cmd);
}

void ros_power_cmd_cb(const corgi_msgs::PowerCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);

    ros_power_cmd = cmd;

    grpc_power_cmd.set_digital(ros_power_cmd.digital);
    grpc_power_cmd.set_signal(ros_power_cmd.signal);
    grpc_power_cmd.set_power(ros_power_cmd.power);
    grpc_power_cmd.set_clean(false);
    grpc_power_cmd.set_trigger(ros_trigger);
    grpc_power_cmd.set_robot_mode((power_msg::ROBOTMODE)ros_power_cmd.robot_mode);

    grpc_power_cmd.mutable_header()->set_seq(ros_power_cmd.header.seq);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_sec(ros_power_cmd.header.stamp.sec);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_usec(ros_power_cmd.header.stamp.nsec);

    grpc_power_cmd_pub->publish(grpc_power_cmd);
}

void grpc_motor_state_cb(const motor_msg::MotorStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_motor_state);

    grpc_motor_state = state;

    std::vector<const motor_msg::MotorState*> grpc_motor_modules = {
        &grpc_motor_state.module_a(),
        &grpc_motor_state.module_b(),
        &grpc_motor_state.module_c(),
        &grpc_motor_state.module_d()
    };

    std::vector<corgi_msgs::MotorState*> ros_motor_modules = {
        &ros_motor_state.module_a,
        &ros_motor_state.module_b,
        &ros_motor_state.module_c,
        &ros_motor_state.module_d
    };

    for (int i = 0; i < 4; i++) {
        ros_motor_modules[i]->theta = grpc_motor_modules[i]->theta();
        ros_motor_modules[i]->beta = grpc_motor_modules[i]->beta();
        ros_motor_modules[i]->velocity_r = grpc_motor_modules[i]->velocity_r();
        ros_motor_modules[i]->velocity_l = grpc_motor_modules[i]->velocity_l();
        ros_motor_modules[i]->torque_r = grpc_motor_modules[i]->torque_r();
        ros_motor_modules[i]->torque_l = grpc_motor_modules[i]->torque_l();
    }

    ros_motor_state.header.seq = grpc_motor_state.header().seq();
    ros_motor_state.header.stamp.sec = grpc_motor_state.header().stamp().sec();
    ros_motor_state.header.stamp.nsec = grpc_motor_state.header().stamp().usec();

    ros_motor_state_pub.publish(ros_motor_state);
}

void grpc_power_state_cb(const power_msg::PowerStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_power_state);

    grpc_power_state = state;

    ros_power_state.digital = grpc_power_state.digital();
    ros_power_state.signal = grpc_power_state.signal();
    ros_power_state.power = grpc_power_state.power();
    ros_power_state.clean = grpc_power_state.clean();
    ros_power_state.robot_mode = grpc_power_state.robot_mode();

    ros_power_state.v_0 = grpc_power_state.v_0();
    ros_power_state.i_0 = grpc_power_state.i_0();
    ros_power_state.v_1 = grpc_power_state.v_1();
    ros_power_state.i_1 = grpc_power_state.i_1();
    ros_power_state.v_2 = grpc_power_state.v_2();
    ros_power_state.i_2 = grpc_power_state.i_2();
    ros_power_state.v_3 = grpc_power_state.v_3();
    ros_power_state.i_3 = grpc_power_state.i_3();
    ros_power_state.v_4 = grpc_power_state.v_4();
    ros_power_state.i_4 = grpc_power_state.i_4();
    ros_power_state.v_5 = grpc_power_state.v_5();
    ros_power_state.i_5 = grpc_power_state.i_5();
    ros_power_state.v_6 = grpc_power_state.v_6();
    ros_power_state.i_6 = grpc_power_state.i_6();
    ros_power_state.v_7 = grpc_power_state.v_7();
    ros_power_state.i_7 = grpc_power_state.i_7();
    ros_power_state.v_8 = grpc_power_state.v_8();
    ros_power_state.i_8 = grpc_power_state.i_8();
    ros_power_state.v_9 = grpc_power_state.v_9();
    ros_power_state.i_9 = grpc_power_state.i_9();
    ros_power_state.v_10 = grpc_power_state.v_10();
    ros_power_state.i_10 = grpc_power_state.i_10();
    ros_power_state.v_11 = grpc_power_state.v_11();
    ros_power_state.i_11 = grpc_power_state.i_11();

    ros_power_state.header.seq = grpc_power_state.header().seq();
    ros_power_state.header.stamp.sec = grpc_power_state.header().stamp().sec();
    ros_power_state.header.stamp.nsec = grpc_power_state.header().stamp().usec();

    ros_power_state_pub.publish(ros_power_state);
}


int main(int argc, char **argv) {
    ROS_INFO_STREAM("Corgi ROS Bridge Starts\n");

    bool debug_mode = false;
    if (argc >= 2 && argv[1] != nullptr) {
        if (strcmp(argv[1], "log") == 0) {
            debug_mode = true;
        }
    }

    ros::init(argc, argv, "corgi_ros_bridge");

    ros::NodeHandle nh;
    ros::Subscriber ros_motor_cmd_sub = nh.subscribe<corgi_msgs::MotorCmdStamped>("motor/command", 1, ros_motor_cmd_cb);
    ros::Subscriber ros_power_cmd_sub = nh.subscribe<corgi_msgs::PowerCmdStamped>("power/command", 1, ros_power_cmd_cb);
    ros::Subscriber ros_trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1, ros_trigger_cb);
    ros_motor_state_pub = nh.advertise<corgi_msgs::MotorStateStamped>("motor/state", 1);
    ros_power_state_pub = nh.advertise<corgi_msgs::PowerStateStamped>("power/state", 1);

    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStateStamped> &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>("motor/state", 1000, grpc_motor_state_cb);
    core::Subscriber<power_msg::PowerStateStamped> &grpc_power_state_sub = nh_.subscribe<power_msg::PowerStateStamped>("power/state", 1000, grpc_power_state_cb);
    grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));
    grpc_power_cmd_pub = &(nh_.advertise<power_msg::PowerCmdStamped>("power/command"));

    core::Rate rate(1000);

    int loop_counter = 0;
    while (ros::ok()) {
        if (debug_mode) ROS_INFO_STREAM("Loop Count: " << loop_counter);

        ros::spinOnce();
        core::spinOnce();

        if (debug_mode) ROS_INFO_STREAM(" ");

        loop_counter++;
        rate.sleep();
    }

    ROS_INFO("Corgi ROS Bridge is killed");

    ros::shutdown();
    
    return 0;
}
