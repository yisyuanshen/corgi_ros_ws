#include <iostream>
#include "NodeHandler.h"
#include "Motor.pb.h"

int main() {
    core::NodeHandler nh;
    core::Rate rate(1000);
    core::Publisher<motor_msg::MotorStateStamped> &motor_state_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");

    motor_msg::MotorStateStamped motor_state;
    
    std::vector<motor_msg::MotorState*> grpc_motor_modules = {
        motor_state.mutable_module_a(),
        motor_state.mutable_module_b(),
        motor_state.mutable_module_c(),
        motor_state.mutable_module_d()
    };

    int loop_counter = 0;
    while (1){
        for (int i=0; i<4; i++){
            grpc_motor_modules[i]->set_theta(30*i);
            grpc_motor_modules[i]->set_beta(30*i);
            grpc_motor_modules[i]->set_current_r(i);
            grpc_motor_modules[i]->set_current_l(i);
        }

        timeval currentTime;
        gettimeofday(&currentTime, nullptr);
        motor_state.mutable_header()->set_seq(loop_counter);
        motor_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
        motor_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
        
        motor_state_pub.publish(motor_state);

        loop_counter++;

        rate.sleep();
    }

    return 0;
}