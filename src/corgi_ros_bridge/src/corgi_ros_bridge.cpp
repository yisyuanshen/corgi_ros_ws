#include "corgi_ros_bridge/LegStamped.h"
#include "corgi_ros_bridge/RobotStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <sys/time.h>
#include <rosgraph_msgs/Clock.h>

#include "force.pb.h"
#include "motor.pb.h"
#include "power.pb.h"
#include "robot.pb.h"
#include <NodeHandler.h>


motor_msg::MotorStamped motor_fb_msg;
force_msg::LegForceStamped force_fb_msg;
robot_msg::State robot_fb_msg;
corgi_ros_bridge::RobotStamped ros_robot_fb_msg;

corgi_ros_bridge::RobotStamped robot_state_msg;

int motor_msg_updated;
int force_msg_updated;
int robot_msg_updated;
int ros_robot_msg_updated;

std::mutex mtx;


void motor_feedback_cb(motor_msg::MotorStamped msg) {
    mtx.lock();
    motor_fb_msg = msg;
    motor_msg_updated = 1;
    mtx.unlock();
}


void force_feedback_cb(force_msg::LegForceStamped msg) {
    mtx.lock();
    force_fb_msg = msg;
    force_msg_updated = 1;
    mtx.unlock();
}


void robot_feedback_cb(robot_msg::State msg) {
    mtx.lock();
    robot_fb_msg = msg;
    robot_msg_updated = 1;
    mtx.unlock();
}


void ros_robot_feedback_cb(corgi_ros_bridge::RobotStamped msg){
    mtx.lock();
    ros_robot_fb_msg = msg;
    ros_robot_msg_updated = 1;
    mtx.unlock();
}


int main(int argc, char **argv) {
    std::cout << "corgi ros bridge started" << std::endl;
    motor_msg_updated = 0;
    force_msg_updated = 0;
    robot_msg_updated = 0;
    ros_robot_msg_updated = 0;

    ros::init(argc, argv, "corgi_ros_bridge");

    ros::NodeHandle nh;
    ros::Subscriber ros_robot_sub = nh.subscribe<corgi_ros_bridge::RobotStamped>("robot/command", 2000, ros_robot_feedback_cb);
    ros::Publisher ros_robot_pub = nh.advertise<corgi_ros_bridge::RobotStamped>("robot/state", 2000);
    ros::Publisher ros_clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 2000);
    
    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStamped> &motor_sub = nh_.subscribe<motor_msg::MotorStamped>("motor/state", 1000, motor_feedback_cb);
    // core::Subscriber<force_msg::LegForceStamped> &force_sub = nh_.subscribe<force_msg::LegForceStamped>("force/state", 1000, force_feedback_cb);
    core::Subscriber<force_msg::LegForceStamped> &force_sub = nh_.subscribe<force_msg::LegForceStamped>("robot/force_state", 1000, force_feedback_cb);
    core::Subscriber<robot_msg::State> &robot_state_sub = nh_.subscribe<robot_msg::State>("robot/state", 1000, robot_feedback_cb);
    core::Publisher<motor_msg::MotorStamped> &motor_pub = nh_.advertise<motor_msg::MotorStamped>("motor/command");
    // core::Publisher<force_msg::LegForceStamped> &force_pub = nh_.advertise<force_msg::LegForceStamped>("force/command");
    core::Publisher<force_msg::LegForceStamped> &force_pub = nh_.advertise<force_msg::LegForceStamped>("robot/force_command");
    
    double rt = 100.0;
    core::Rate rate(rt);
    
    int count = 0;
    while (ros::ok()) {        
        ROS_INFO_STREAM("Loop Count: " << count);

        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock.fromSec(count/rt);
        ros_clock_pub.publish(clock_msg);

        ros::spinOnce();
        core::spinOnce();
        
        if (ros_robot_msg_updated){
            if (ros_robot_fb_msg.msg_type == "force"){
                force_msg::LegForceStamped force_cmd_msg;
                force_msg::LegForce force_cmd;
                force_msg::Impedance imp_cmd;

                mtx.lock();

                force_cmd.set_pose_x(ros_robot_fb_msg.A_LF.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.A_LF.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.A_LF.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.A_LF.force.force_y);
                imp_cmd.set_m_x(ros_robot_fb_msg.A_LF.impedance.M_x);
                imp_cmd.set_m_y(ros_robot_fb_msg.A_LF.impedance.M_y);
                imp_cmd.set_k0_x(ros_robot_fb_msg.A_LF.impedance.K0_x);
                imp_cmd.set_k0_y(ros_robot_fb_msg.A_LF.impedance.K0_y);
                imp_cmd.set_d_x(ros_robot_fb_msg.A_LF.impedance.D_x);
                imp_cmd.set_d_y(ros_robot_fb_msg.A_LF.impedance.D_y);
                imp_cmd.set_adaptive_kp_x(ros_robot_fb_msg.A_LF.impedance.adaptive_kp_x);
                imp_cmd.set_adaptive_kp_y(ros_robot_fb_msg.A_LF.impedance.adaptive_kp_y);
                imp_cmd.set_adaptive_ki_x(ros_robot_fb_msg.A_LF.impedance.adaptive_ki_x);
                imp_cmd.set_adaptive_ki_y(ros_robot_fb_msg.A_LF.impedance.adaptive_ki_y);
                imp_cmd.set_adaptive_kd_x(ros_robot_fb_msg.A_LF.impedance.adaptive_kd_x);
                imp_cmd.set_adaptive_kd_y(ros_robot_fb_msg.A_LF.impedance.adaptive_kd_y);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                force_cmd.set_pose_x(ros_robot_fb_msg.B_RF.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.B_RF.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.B_RF.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.B_RF.force.force_y);
                imp_cmd.set_m_x(ros_robot_fb_msg.B_RF.impedance.M_x);
                imp_cmd.set_m_y(ros_robot_fb_msg.B_RF.impedance.M_y);
                imp_cmd.set_k0_x(ros_robot_fb_msg.B_RF.impedance.K0_x);
                imp_cmd.set_k0_y(ros_robot_fb_msg.B_RF.impedance.K0_y);
                imp_cmd.set_d_x(ros_robot_fb_msg.B_RF.impedance.D_x);
                imp_cmd.set_d_y(ros_robot_fb_msg.B_RF.impedance.D_y);
                imp_cmd.set_adaptive_kp_x(ros_robot_fb_msg.B_RF.impedance.adaptive_kp_x);
                imp_cmd.set_adaptive_kp_y(ros_robot_fb_msg.B_RF.impedance.adaptive_kp_y);
                imp_cmd.set_adaptive_ki_x(ros_robot_fb_msg.B_RF.impedance.adaptive_ki_x);
                imp_cmd.set_adaptive_ki_y(ros_robot_fb_msg.B_RF.impedance.adaptive_ki_y);
                imp_cmd.set_adaptive_kd_x(ros_robot_fb_msg.B_RF.impedance.adaptive_kd_x);
                imp_cmd.set_adaptive_kd_y(ros_robot_fb_msg.B_RF.impedance.adaptive_kd_y);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                force_cmd.set_pose_x(ros_robot_fb_msg.C_RH.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.C_RH.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.C_RH.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.C_RH.force.force_y);
                imp_cmd.set_m_x(ros_robot_fb_msg.C_RH.impedance.M_x);
                imp_cmd.set_m_y(ros_robot_fb_msg.C_RH.impedance.M_y);
                imp_cmd.set_k0_x(ros_robot_fb_msg.C_RH.impedance.K0_x);
                imp_cmd.set_k0_y(ros_robot_fb_msg.C_RH.impedance.K0_y);
                imp_cmd.set_d_x(ros_robot_fb_msg.C_RH.impedance.D_x);
                imp_cmd.set_d_y(ros_robot_fb_msg.C_RH.impedance.D_y);
                imp_cmd.set_adaptive_kp_x(ros_robot_fb_msg.C_RH.impedance.adaptive_kp_x);
                imp_cmd.set_adaptive_kp_y(ros_robot_fb_msg.C_RH.impedance.adaptive_kp_y);
                imp_cmd.set_adaptive_ki_x(ros_robot_fb_msg.C_RH.impedance.adaptive_ki_x);
                imp_cmd.set_adaptive_ki_y(ros_robot_fb_msg.C_RH.impedance.adaptive_ki_y);
                imp_cmd.set_adaptive_kd_x(ros_robot_fb_msg.C_RH.impedance.adaptive_kd_x);
                imp_cmd.set_adaptive_kd_y(ros_robot_fb_msg.C_RH.impedance.adaptive_kd_y);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                force_cmd.set_pose_x(ros_robot_fb_msg.D_LH.force.pose_x);
                force_cmd.set_pose_y(ros_robot_fb_msg.D_LH.force.pose_y);
                force_cmd.set_force_x(ros_robot_fb_msg.D_LH.force.force_x);
                force_cmd.set_force_y(ros_robot_fb_msg.D_LH.force.force_y);
                imp_cmd.set_m_x(ros_robot_fb_msg.D_LH.impedance.M_x);
                imp_cmd.set_m_y(ros_robot_fb_msg.D_LH.impedance.M_y);
                imp_cmd.set_k0_x(ros_robot_fb_msg.D_LH.impedance.K0_x);
                imp_cmd.set_k0_y(ros_robot_fb_msg.D_LH.impedance.K0_y);
                imp_cmd.set_d_x(ros_robot_fb_msg.D_LH.impedance.D_x);
                imp_cmd.set_d_y(ros_robot_fb_msg.D_LH.impedance.D_y);
                imp_cmd.set_adaptive_kp_x(ros_robot_fb_msg.D_LH.impedance.adaptive_kp_x);
                imp_cmd.set_adaptive_kp_y(ros_robot_fb_msg.D_LH.impedance.adaptive_kp_y);
                imp_cmd.set_adaptive_ki_x(ros_robot_fb_msg.D_LH.impedance.adaptive_ki_x);
                imp_cmd.set_adaptive_ki_y(ros_robot_fb_msg.D_LH.impedance.adaptive_ki_y);
                imp_cmd.set_adaptive_kd_x(ros_robot_fb_msg.D_LH.impedance.adaptive_kd_x);
                imp_cmd.set_adaptive_kd_y(ros_robot_fb_msg.D_LH.impedance.adaptive_kd_y);
                force_cmd_msg.add_force()->CopyFrom(force_cmd);
                force_cmd_msg.add_impedance()->CopyFrom(imp_cmd);

                mtx.unlock();

                force_pub.publish(force_cmd_msg);
                ROS_INFO_STREAM("Force Command Published");
            }
            else if (ros_robot_fb_msg.msg_type == "motor"){
                motor_msg::MotorStamped motor_cmd_msg;
                motor_msg::Motor motor_cmd;
                motor_msg::LegAngle leg_cmd;

                mtx.lock();

                motor_cmd.set_angle(ros_robot_fb_msg.A_LF.motor_r.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.A_LF.motor_r.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.A_LF.motor_r.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.A_LF.motor_r.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.A_LF.motor_r.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.A_LF.motor_r.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.A_LF.motor_l.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.A_LF.motor_l.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.A_LF.motor_l.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.A_LF.motor_l.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.A_LF.motor_l.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.A_LF.motor_l.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.A_LF.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.A_LF.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.B_RF.motor_r.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.B_RF.motor_r.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.B_RF.motor_r.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.B_RF.motor_r.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.B_RF.motor_r.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.B_RF.motor_r.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.B_RF.motor_l.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.B_RF.motor_l.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.B_RF.motor_l.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.B_RF.motor_l.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.B_RF.motor_l.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.B_RF.motor_l.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.B_RF.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.B_RF.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.C_RH.motor_r.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.C_RH.motor_r.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.C_RH.motor_r.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.C_RH.motor_r.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.C_RH.motor_r.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.C_RH.motor_r.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.C_RH.motor_l.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.C_RH.motor_l.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.C_RH.motor_l.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.C_RH.motor_l.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.C_RH.motor_l.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.C_RH.motor_l.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.C_RH.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.C_RH.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.D_LH.motor_r.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.D_LH.motor_r.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.D_LH.motor_r.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.D_LH.motor_r.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.D_LH.motor_r.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.D_LH.motor_r.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                motor_cmd.set_angle(ros_robot_fb_msg.D_LH.motor_l.angle);
                // motor_cmd.set_twist(ros_robot_fb_msg.D_LH.motor_l.twist);
                motor_cmd.set_torque(ros_robot_fb_msg.D_LH.motor_l.torque);
                motor_cmd.set_kp(ros_robot_fb_msg.D_LH.motor_l.kp);
                motor_cmd.set_ki(ros_robot_fb_msg.D_LH.motor_l.ki);
                motor_cmd.set_kd(ros_robot_fb_msg.D_LH.motor_l.kd);
                motor_cmd_msg.add_motors()->CopyFrom(motor_cmd);

                leg_cmd.set_theta(ros_robot_fb_msg.D_LH.theta);
                leg_cmd.set_beta(ros_robot_fb_msg.D_LH.beta);
                motor_cmd_msg.add_legs()->CopyFrom(leg_cmd);

                mtx.unlock();

                motor_pub.publish(motor_cmd_msg);
                ROS_INFO_STREAM("Motor Command Published");
            }

            ros_robot_msg_updated = 0;
        }
        
        
        if (robot_msg_updated || force_msg_updated || motor_msg_updated){
            mtx.lock();

            if (robot_msg_updated){
                robot_state_msg.pose.position.x = robot_fb_msg.pose().position().x();
                robot_state_msg.pose.position.y = robot_fb_msg.pose().position().y();
                robot_state_msg.pose.position.z = robot_fb_msg.pose().position().z();
                robot_state_msg.pose.orientation.x = robot_fb_msg.pose().orientation().x();
                robot_state_msg.pose.orientation.y = robot_fb_msg.pose().orientation().y();
                robot_state_msg.pose.orientation.z = robot_fb_msg.pose().orientation().z();
                robot_state_msg.pose.orientation.w = robot_fb_msg.pose().orientation().w();
                robot_state_msg.twist.linear.x = robot_fb_msg.twist().linear().x();
                robot_state_msg.twist.linear.y = robot_fb_msg.twist().linear().y();
                robot_state_msg.twist.linear.z = robot_fb_msg.twist().linear().z();
                robot_state_msg.twist.angular.x = robot_fb_msg.twist().angular().x();
                robot_state_msg.twist.angular.y = robot_fb_msg.twist().angular().y();
                robot_state_msg.twist.angular.z = robot_fb_msg.twist().angular().z();

                robot_msg_updated = 0;
                ROS_INFO_STREAM("Robot State Received");
            }

            if (force_msg_updated){
                if (force_fb_msg.force().size() == 4){
                    robot_state_msg.A_LF.force.pose_x = force_fb_msg.force(0).pose_x();
                    robot_state_msg.A_LF.force.pose_y = force_fb_msg.force(0).pose_y();
                    robot_state_msg.A_LF.force.force_x = force_fb_msg.force(0).force_x();
                    robot_state_msg.A_LF.force.force_y = force_fb_msg.force(0).force_y();
                    robot_state_msg.B_RF.force.pose_x = force_fb_msg.force(1).pose_x();
                    robot_state_msg.B_RF.force.pose_y = force_fb_msg.force(1).pose_y();
                    robot_state_msg.B_RF.force.force_x = force_fb_msg.force(1).force_x();
                    robot_state_msg.B_RF.force.force_y = force_fb_msg.force(1).force_y();
                    robot_state_msg.C_RH.force.pose_x = force_fb_msg.force(2).pose_x();
                    robot_state_msg.C_RH.force.pose_y = force_fb_msg.force(2).pose_y();
                    robot_state_msg.C_RH.force.force_x = force_fb_msg.force(2).force_x();
                    robot_state_msg.C_RH.force.force_y = force_fb_msg.force(2).force_y();
                    robot_state_msg.D_LH.force.pose_x = force_fb_msg.force(3).pose_x();
                    robot_state_msg.D_LH.force.pose_y = force_fb_msg.force(3).pose_y();
                    robot_state_msg.D_LH.force.force_x = force_fb_msg.force(3).force_x();
                    robot_state_msg.D_LH.force.force_y = force_fb_msg.force(3).force_y();
                    
                    ROS_INFO_STREAM("Force State Received");
                }
                force_msg_updated = 0;
             }

            if (motor_msg_updated){
                if (motor_fb_msg.motors().size() == 8){
                    robot_state_msg.A_LF.motor_r.angle = motor_fb_msg.motors(0).angle();
                    robot_state_msg.A_LF.motor_r.torque = motor_fb_msg.motors(0).torque();
                    robot_state_msg.A_LF.motor_r.twist = motor_fb_msg.motors(0).twist();
                    robot_state_msg.A_LF.motor_r.kp = motor_fb_msg.motors(0).kp();
                    robot_state_msg.A_LF.motor_r.ki = motor_fb_msg.motors(0).ki();
                    robot_state_msg.A_LF.motor_r.kd = motor_fb_msg.motors(0).kd();
                    robot_state_msg.A_LF.motor_l.angle = motor_fb_msg.motors(1).angle();
                    robot_state_msg.A_LF.motor_l.torque = motor_fb_msg.motors(1).torque();
                    robot_state_msg.A_LF.motor_l.twist = motor_fb_msg.motors(1).twist();
                    robot_state_msg.A_LF.motor_l.kp = motor_fb_msg.motors(1).kp();
                    robot_state_msg.A_LF.motor_l.ki = motor_fb_msg.motors(1).ki();
                    robot_state_msg.A_LF.motor_l.kd = motor_fb_msg.motors(1).kd();
                    // robot_state_msg.A_LF.theta = motor_fb_msg.legs(0).theta();
                    // robot_state_msg.A_LF.beta = motor_fb_msg.legs(0).beta();

                    robot_state_msg.B_RF.motor_r.angle = motor_fb_msg.motors(2).angle();
                    robot_state_msg.B_RF.motor_r.torque = motor_fb_msg.motors(2).torque();
                    robot_state_msg.B_RF.motor_r.twist = motor_fb_msg.motors(2).twist();
                    robot_state_msg.B_RF.motor_r.kp = motor_fb_msg.motors(2).kp();
                    robot_state_msg.B_RF.motor_r.ki = motor_fb_msg.motors(2).ki();
                    robot_state_msg.B_RF.motor_r.kd = motor_fb_msg.motors(2).kd();
                    robot_state_msg.B_RF.motor_l.angle = motor_fb_msg.motors(3).angle();
                    robot_state_msg.B_RF.motor_l.torque = motor_fb_msg.motors(3).torque();
                    robot_state_msg.B_RF.motor_l.twist = motor_fb_msg.motors(3).twist();
                    robot_state_msg.B_RF.motor_l.kp = motor_fb_msg.motors(3).kp();
                    robot_state_msg.B_RF.motor_l.ki = motor_fb_msg.motors(3).ki();
                    robot_state_msg.B_RF.motor_l.kd = motor_fb_msg.motors(3).kd();
                    // robot_state_msg.B_RF.theta = motor_fb_msg.legs(1).theta();
                    // robot_state_msg.B_RF.beta = motor_fb_msg.legs(1).beta();

                    robot_state_msg.C_RH.motor_r.angle = motor_fb_msg.motors(4).angle();
                    robot_state_msg.C_RH.motor_r.torque = motor_fb_msg.motors(4).torque();
                    robot_state_msg.C_RH.motor_r.twist = motor_fb_msg.motors(4).twist();
                    robot_state_msg.C_RH.motor_r.kp = motor_fb_msg.motors(4).kp();
                    robot_state_msg.C_RH.motor_r.ki = motor_fb_msg.motors(4).ki();
                    robot_state_msg.C_RH.motor_r.kd = motor_fb_msg.motors(4).kd();
                    robot_state_msg.C_RH.motor_l.angle = motor_fb_msg.motors(5).angle();
                    robot_state_msg.C_RH.motor_l.torque = motor_fb_msg.motors(5).torque();
                    robot_state_msg.C_RH.motor_l.twist = motor_fb_msg.motors(5).twist();
                    robot_state_msg.C_RH.motor_l.kp = motor_fb_msg.motors(5).kp();
                    robot_state_msg.C_RH.motor_l.ki = motor_fb_msg.motors(5).ki();
                    robot_state_msg.C_RH.motor_l.kd = motor_fb_msg.motors(5).kd();
                    // robot_state_msg.C_RH.theta = motor_fb_msg.legs(2).theta();
                    // robot_state_msg.C_RH.beta = motor_fb_msg.legs(2).beta();

                    robot_state_msg.D_LH.motor_r.angle = motor_fb_msg.motors(6).angle();
                    robot_state_msg.D_LH.motor_r.torque = motor_fb_msg.motors(6).torque();
                    robot_state_msg.D_LH.motor_r.twist = motor_fb_msg.motors(6).twist();
                    robot_state_msg.D_LH.motor_r.kp = motor_fb_msg.motors(6).kp();
                    robot_state_msg.D_LH.motor_r.ki = motor_fb_msg.motors(6).ki();
                    robot_state_msg.D_LH.motor_r.kd = motor_fb_msg.motors(6).kd();
                    robot_state_msg.D_LH.motor_l.angle = motor_fb_msg.motors(7).angle();
                    robot_state_msg.D_LH.motor_l.torque = motor_fb_msg.motors(7).torque();
                    robot_state_msg.D_LH.motor_l.twist = motor_fb_msg.motors(7).twist();
                    robot_state_msg.D_LH.motor_l.kp = motor_fb_msg.motors(7).kp();
                    robot_state_msg.D_LH.motor_l.ki = motor_fb_msg.motors(7).ki();
                    robot_state_msg.D_LH.motor_l.kd = motor_fb_msg.motors(7).kd();
                    // robot_state_msg.D_LH.theta = motor_fb_msg.legs(3).theta();
                    // robot_state_msg.D_LH.beta = motor_fb_msg.legs(3).beta();

                    ROS_INFO_STREAM("Motor State Received");
                }
                motor_msg_updated = 0;
            }

            mtx.unlock();
            
            ros_robot_pub.publish(robot_state_msg);
        }
        
        count += 1;

        rate.sleep();
    }

    ROS_INFO("Shutting down the node...");

    return 0;
}
