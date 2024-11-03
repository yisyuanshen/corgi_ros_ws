#include <iostream>
#include <mutex>
#include "ros/ros.h"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"


std::mutex mutex_motor_state;
std::mutex mutex_power_state;

motor_msg::MotorCmdStamped      motor_cmd;
power_msg::PowerCmdStamped      power_cmd;
motor_msg::MotorStateStamped    motor_state;
power_msg::PowerStateStamped    power_state;


void motor_cmd_cb(const motor_msg::MotorCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_motor_state);

    std::vector<motor_msg::MotorState*> motor_states = {
        motor_state.mutable_module_a(),
        motor_state.mutable_module_b(),
        motor_state.mutable_module_c(),
        motor_state.mutable_module_d()
    };

    std::vector<const motor_msg::MotorCmd*> motor_cmds = {
        &cmd.module_a(),
        &cmd.module_b(),
        &cmd.module_c(),
        &cmd.module_d()
    };

    for (int i = 0; i < 4; i++) {
        motor_states[i]->set_theta(motor_cmds[i]->theta());
        motor_states[i]->set_beta(motor_cmds[i]->beta());
        motor_states[i]->set_current_r(1);
        motor_states[i]->set_current_l(1);
    }

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    motor_state.mutable_header()->set_seq(cmd.header().seq());
    motor_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    motor_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}

void power_cmd_cb(const power_msg::PowerCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_power_state);

    power_state.set_digital(cmd.digital());
    power_state.set_power(cmd.power());
    // std::cout << cmd.motor_mode() << std::endl;
    power_state.set_motor_mode((power_msg::MOTORMODE)cmd.motor_mode());


    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    power_state.mutable_header()->set_seq(cmd.header().seq());
    power_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    power_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_virtual_agent");

    core::NodeHandler nh_;
    core::Publisher<motor_msg::MotorStateStamped> &motor_state_pub = nh_.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Publisher<power_msg::PowerStateStamped> &power_state_pub = nh_.advertise<power_msg::PowerStateStamped>("power/state");
    core::Subscriber<motor_msg::MotorCmdStamped> &motor_cmd_sub = nh_.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);
    core::Subscriber<power_msg::PowerCmdStamped> &power_cmd_sub = nh_.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_cmd_cb);

    core::Rate rate(1000);

    while (ros::ok()) {
        core::spinOnce();

        {
            std::lock_guard<std::mutex> lock(mutex_motor_state);
            motor_state.mutable_header()->set_seq(1);
            motor_state_pub.publish(motor_state);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_power_state);
            power_state_pub.publish(power_state);
        }

        rate.sleep();
    }

    ros::shutdown();


    return 0;
}