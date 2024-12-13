#include <iostream>
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "corgi_sim/set_int.h"
#include "corgi_sim/set_float.h"
#include "corgi_sim/Float64Stamped.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"

corgi_msgs::MotorCmdStamped motor_cmd;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::TriggerStamped trigger;

double AR_phi = 0.0;
double AL_phi = 0.0;
double BR_phi = 0.0;
double BL_phi = 0.0;
double CR_phi = 0.0;
double CL_phi = 0.0;
double DR_phi = 0.0;
double DL_phi = 0.0;

corgi_sim::set_float AR_motor_srv;
corgi_sim::set_float AL_motor_srv;
corgi_sim::set_float BR_motor_srv;
corgi_sim::set_float BL_motor_srv;
corgi_sim::set_float CR_motor_srv;
corgi_sim::set_float CL_motor_srv;
corgi_sim::set_float DR_motor_srv;
corgi_sim::set_float DL_motor_srv;

corgi_sim::set_int time_step_srv;

rosgraph_msgs::Clock simulationClock;

void motor_cmd_cb(const corgi_msgs::MotorCmdStamped cmd) {motor_cmd = cmd;}

void AR_encoder_cb(corgi_sim::Float64Stamped phi) { AR_phi = phi.data; }
void AL_encoder_cb(corgi_sim::Float64Stamped phi) { AL_phi = phi.data; }
void BR_encoder_cb(corgi_sim::Float64Stamped phi) { BR_phi = phi.data; }
void BL_encoder_cb(corgi_sim::Float64Stamped phi) { BL_phi = phi.data; }
void CR_encoder_cb(corgi_sim::Float64Stamped phi) { CR_phi = phi.data; }
void CL_encoder_cb(corgi_sim::Float64Stamped phi) { CL_phi = phi.data; }
void DR_encoder_cb(corgi_sim::Float64Stamped phi) { DR_phi = phi.data; }
void DL_encoder_cb(corgi_sim::Float64Stamped phi) { DL_phi = phi.data; }

void AR_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_a.torque_r = trq.data; }
void AL_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_a.torque_l = trq.data; }
void BR_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_b.torque_r = trq.data; }
void BL_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_b.torque_l = trq.data; }
void CR_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_c.torque_r = trq.data; }
void CL_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_c.torque_l = trq.data; }
void DR_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_d.torque_r = trq.data; }
void DL_torque_cb(corgi_sim::Float64Stamped trq) { motor_state.module_d.torque_l = trq.data; }

void phi2tb(double phi_r, double phi_l, double &theta, double &beta){
    theta = (phi_l - phi_r) / 2.0 + 17 / 180.0 * M_PI;
    beta  = (phi_l + phi_r) / 2.0;
}

void tb2phi(double theta, double beta, double &phi_r, double &phi_l){
    double theta_0 = 17 / 180.0 * M_PI;
    if (theta < theta_0) {theta = theta_0;}
    phi_r = beta - theta + theta_0;
    phi_l = beta + theta - theta_0;
}


int main(int argc, char **argv) {
    ROS_INFO("Corgi Simulation Starts\n");

    ros::init(argc, argv, "corgi_sim");

    ros::NodeHandle nh;

    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
    
    ros::service::waitForService("/robot/time_step");
    ros::spinOnce();

    ros::ServiceClient time_step_client = nh.serviceClient<corgi_sim::set_int>("robot/time_step");

    ros::ServiceClient AR_motor_client = nh.serviceClient<corgi_sim::set_float>("lf_right_motor/set_position");
    ros::ServiceClient AL_motor_client = nh.serviceClient<corgi_sim::set_float>("lf_left_motor/set_position");
    ros::ServiceClient BR_motor_client = nh.serviceClient<corgi_sim::set_float>("rf_right_motor/set_position");
    ros::ServiceClient BL_motor_client = nh.serviceClient<corgi_sim::set_float>("rf_left_motor/set_position");
    ros::ServiceClient CR_motor_client = nh.serviceClient<corgi_sim::set_float>("rh_right_motor/set_position");
    ros::ServiceClient CL_motor_client = nh.serviceClient<corgi_sim::set_float>("rh_left_motor/set_position");
    ros::ServiceClient DR_motor_client = nh.serviceClient<corgi_sim::set_float>("lh_right_motor/set_position");
    ros::ServiceClient DL_motor_client = nh.serviceClient<corgi_sim::set_float>("lh_left_motor/set_position");
    
    ros::Subscriber AR_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("lf_right_motor_sensor/value", 1, AR_encoder_cb);
    ros::Subscriber AL_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("lf_left_motor_sensor/value" , 1, AL_encoder_cb);
    ros::Subscriber BR_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("rf_right_motor_sensor/value", 1, BR_encoder_cb);
    ros::Subscriber BL_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("rf_left_motor_sensor/value" , 1, BL_encoder_cb);
    ros::Subscriber CR_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("rh_right_motor_sensor/value", 1, CR_encoder_cb);
    ros::Subscriber CL_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("rh_left_motor_sensor/value" , 1, CL_encoder_cb);
    ros::Subscriber DR_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("lh_right_motor_sensor/value", 1, DR_encoder_cb);
    ros::Subscriber DL_encoder_sub = nh.subscribe<corgi_sim::Float64Stamped>("lh_left_motor_sensor/value" , 1, DL_encoder_cb);
    
    ros::Subscriber AR_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("lf_right_motor/torque_feedback", 1, AR_torque_cb);
    ros::Subscriber AL_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("lf_left_motor/torque_feedback" , 1, AL_torque_cb);
    ros::Subscriber BR_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("rf_right_motor/torque_feedback", 1, BR_torque_cb);
    ros::Subscriber BL_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("rf_left_motor/torque_feedback" , 1, BL_torque_cb);
    ros::Subscriber CR_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("rh_right_motor/torque_feedback", 1, CR_torque_cb);
    ros::Subscriber CL_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("rh_left_motor/torque_feedback" , 1, CL_torque_cb);
    ros::Subscriber DR_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("lh_right_motor/torque_feedback", 1, DR_torque_cb);
    ros::Subscriber DL_torque_sub = nh.subscribe<corgi_sim::Float64Stamped>("lh_left_motor/torque_feedback" , 1, DL_torque_cb);
    
    ros::Subscriber motor_cmd_sub = nh.subscribe<corgi_msgs::MotorCmdStamped>("motor/command", 1, motor_cmd_cb);
    ros::Publisher motor_state_pub = nh.advertise<corgi_msgs::MotorStateStamped>("motor/state", 1000);
    ros::Publisher trigger_pub = nh.advertise<corgi_msgs::TriggerStamped>("trigger", 1000);
    
    ros::WallRate rate(1000);
    
    trigger.enable = true;
    time_step_srv.request.value = 1;

    int loop_counter = 0;
    while (ros::ok() && time_step_client.call(time_step_srv)){
        ros::spinOnce();

        tb2phi(motor_cmd.module_a.theta, motor_cmd.module_a.beta, AR_motor_srv.request.value, AL_motor_srv.request.value);
        tb2phi(motor_cmd.module_b.theta, motor_cmd.module_b.beta, BR_motor_srv.request.value, BL_motor_srv.request.value);
        tb2phi(motor_cmd.module_c.theta, motor_cmd.module_c.beta, CR_motor_srv.request.value, CL_motor_srv.request.value);
        tb2phi(motor_cmd.module_d.theta, motor_cmd.module_d.beta, DR_motor_srv.request.value, DL_motor_srv.request.value);

        AR_motor_client.call(AR_motor_srv);
        AL_motor_client.call(AL_motor_srv);
        BR_motor_client.call(BR_motor_srv);
        BL_motor_client.call(BL_motor_srv);
        CR_motor_client.call(CR_motor_srv);
        CL_motor_client.call(CL_motor_srv);
        DR_motor_client.call(DR_motor_srv);
        DL_motor_client.call(DL_motor_srv);

        phi2tb(AR_phi, AL_phi, motor_state.module_a.theta, motor_state.module_a.beta);
        phi2tb(BR_phi, BL_phi, motor_state.module_b.theta, motor_state.module_b.beta);
        phi2tb(CR_phi, CL_phi, motor_state.module_c.theta, motor_state.module_c.beta);
        phi2tb(DR_phi, DL_phi, motor_state.module_d.theta, motor_state.module_d.beta);

        motor_state.header.seq = loop_counter;

        motor_state_pub.publish(motor_state);
        trigger_pub.publish(trigger);

        double clock = loop_counter*0.001;
        simulationClock.clock.sec = (int)clock;
        simulationClock.clock.nsec = round(1000 * (clock - simulationClock.clock.sec)) * 1.0e+6;
        clock_pub.publish(simulationClock);

        loop_counter++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}
