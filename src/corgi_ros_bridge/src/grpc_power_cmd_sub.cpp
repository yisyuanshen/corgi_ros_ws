#include "NodeHandler.h"
#include "Power.pb.h"

std::mutex mutex_;

void power_cmd_cb(power_msg::PowerCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_);

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);

    std::cout << "= = = Power Cmd Received = = =" << std::endl
              << "Current Time: " << currentTime.tv_sec << "." << currentTime.tv_usec << std::endl
              << "Time Stamp: " << cmd.header().stamp().sec() << "." << cmd.header().stamp().usec() << std::endl
              << "Seq: " << cmd.header().seq() << std::endl
              << "Digital: " << cmd.digital() << "; Power: " << cmd.power() << "; Motor Mode: " << cmd.motor_mode() << std::endl
              << std::endl;
}

int main() {
    core::NodeHandler nh;
    core::Rate rate(1000);
    core::Subscriber<power_msg::PowerCmdStamped> &power_cmd_sub = nh.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_cmd_cb);
    
    while (1){
        core::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
