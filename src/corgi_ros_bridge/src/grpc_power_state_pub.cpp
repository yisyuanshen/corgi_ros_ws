#include <iostream>
#include "NodeHandler.h"
#include "Power.pb.h"

int main() {
    core::NodeHandler nh;
    core::Rate rate(1000);
    core::Publisher<power_msg::PowerStateStamped> &power_state_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");

    power_msg::PowerStateStamped power_state;

    int loop_counter = 0;
    while (1){
        int motor_mode = 1;
        power_state.set_digital(true);
        power_state.set_power(true);
        power_state.set_motor_mode((power_msg::MOTORMODE)motor_mode);

        double v = 47.5;
        double i = 10.5;
        power_state.set_v_0(v);
        power_state.set_i_0(i);
        power_state.set_v_1(v);
        power_state.set_i_1(i);
        power_state.set_v_2(v);
        power_state.set_i_2(i);
        power_state.set_v_3(v);
        power_state.set_i_3(i);
        power_state.set_v_4(v);
        power_state.set_i_4(i);
        power_state.set_v_5(v);
        power_state.set_i_5(i);
        power_state.set_v_6(v);
        power_state.set_i_6(i);
        power_state.set_v_7(v);
        power_state.set_i_7(i);
        power_state.set_v_8(v);
        power_state.set_i_8(i);
        power_state.set_v_9(v);
        power_state.set_i_9(i);
        power_state.set_v_10(v);
        power_state.set_i_10(i);
        power_state.set_v_11(v);
        power_state.set_i_11(i);

        timeval currentTime;
        gettimeofday(&currentTime, nullptr);
        power_state.mutable_header()->set_seq(loop_counter);
        power_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
        power_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);

        power_state_pub.publish(power_state);

        loop_counter++;

        rate.sleep();
    }

    return 0;
}