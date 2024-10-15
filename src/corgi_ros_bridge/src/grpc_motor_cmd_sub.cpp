#include "NodeHandler.h"
#include "Motor.pb.h"

std::mutex mutex_;

void motor_cmd_cb(motor_msg::MotorCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    timeval currentTime;
    gettimeofday(&currentTime, nullptr);

    std::cout << "= = = Motor Cmd Received = = =" << std::endl
              << "Current Time: " << currentTime.tv_sec << "." << currentTime.tv_usec << std::endl
              << "Time Stamp: " << cmd.header().stamp().sec() << "." << cmd.header().stamp().usec() << std::endl
              << "Seq: " << cmd.header().seq() << std::endl
              << "Module A: theta = " << cmd.module_a().theta() << "; beta = " << cmd.module_a().beta()
              << "; kp = " << cmd.module_a().kp() << "; ki = " << cmd.module_a().ki() << "; kd = " << cmd.module_a().kd() << std::endl
              << "Module B: theta = " << cmd.module_b().theta() << "; beta = " << cmd.module_b().beta()
              << "; kp = " << cmd.module_b().kp() << "; ki = " << cmd.module_b().ki() << "; kd = " << cmd.module_b().kd() << std::endl
              << "Module C: theta = " << cmd.module_c().theta() << "; beta = " << cmd.module_c().beta()
              << "; kp = " << cmd.module_c().kp() << "; ki = " << cmd.module_c().ki() << "; kd = " << cmd.module_c().kd() << std::endl
              << "Module D: theta = " << cmd.module_d().theta() << "; beta = " << cmd.module_d().beta()
              << "; kp = " << cmd.module_d().kp() << "; ki = " << cmd.module_d().ki() << "; kd = " << cmd.module_d().kd() << std::endl
              << std::endl;
}

int main() {
    core::NodeHandler nh;
    core::Rate rate(1000);
    core::Subscriber<motor_msg::MotorCmdStamped> &motor_cmd_sub = nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);
    
    while (1){
        core::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
