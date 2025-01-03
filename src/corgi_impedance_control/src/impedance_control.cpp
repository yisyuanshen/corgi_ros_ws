#include "impedance_control.hpp"


void imp_cmd_cb(const corgi_msgs::ImpedanceCmdStamped cmd){
    imp_cmd = cmd;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}


int main(int argc, char **argv) {

    ROS_INFO("Impedance Control Starts\n");

    ros::init(argc, argv, "impedance_control");

    ros::NodeHandle nh;
    ros::Subscriber imp_cmd_sub = nh.subscribe<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000, imp_cmd_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    
    ros::Rate rate(1000);


    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };


    


    while (ros::ok()) {
        ros::spinOnce();








        for (int i=0; i<4; i++){
            printf("-\n");
            printf("imp_cmd: P = [%lf, %lf], F = [%lf, %lf]\n", imp_cmd_modules[i]->Px, imp_cmd_modules[i]->Py, imp_cmd_modules[i]->Fx, imp_cmd_modules[i]->Fy);
            printf("motor_state: P = [%lf, %lf]\n", motor_state_modules[i]->theta, motor_state_modules[i]->beta);
            printf("force_state: F = [%lf, %lf]\n", force_state_modules[i]->Fx, force_state_modules[i]->Fy);
            // printf("motor_state: TB = [%lf, %lf]\n", motor_state_modules[i]->theta, motor_state_modules[i]->beta);
        }
        
        rate.sleep();
    }

    ros::shutdown();

    return 0;
}