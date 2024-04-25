#include "corgi_ros_bridge/LegStamped.h"
#include "corgi_ros_bridge/RobotStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <mutex>
#include <ros/ros.h>

#include "math.h"
#include "force.pb.h"
#include "motor.pb.h"
#include "power.pb.h"
#include "robot.pb.h"
#include <NodeHandler.h>
#include <sys/time.h>

motor_msg::MotorStamped motor_fb_msg;
force_msg::LegForceStamped force_fb_msg;
robot_msg::State robot_state_fb_msg;
corgi_ros_bridge::RobotStamped force_ros_msg;

int motor_msg_updated;
int force_msg_updated;
int robot_state_msg_update;
int force_ros_msg_updated;

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

void robot_state_feedback_cb(robot_msg::State msg) {
  mtx.lock();
  robot_state_fb_msg = msg;
  robot_state_msg_update = 1;
  mtx.unlock();
}

void ros_force_cb(corgi_ros_bridge::RobotStamped msg){
  mtx.lock();
  ROS_INFO("Received Force Message ...");
  force_ros_msg = msg;
  force_ros_msg_updated = 1;
  mtx.unlock();
}

int main(int argc, char **argv) {
  std::cout << "ros corgi bridge started" << std::endl;
  motor_msg_updated = 0;
  ros::init(argc, argv, "ros_corgi_bridge");

  ros::NodeHandle nh;
  // ros::Publisher ros_legpub_ =
  //     nh.advertise<corgi_ros_bridge::RobotStamped>("Robot_legs", 2000);
  ros::Publisher ros_robot_state_pub =
      nh.advertise<corgi_ros_bridge::RobotStamped>("robot/state", 2000);
  ros::Subscriber ros_force_sub =
      nh.subscribe<corgi_ros_bridge::RobotStamped>("force/force_command", 2000,
                                                   ros_force_cb);
  ros::Rate rt(500);

  core::NodeHandler nh_;
  core::Publisher<motor_msg::MotorStamped> &motor_pub =
      nh_.advertise<motor_msg::MotorStamped>("motor/command");
  core::Subscriber<motor_msg::MotorStamped> &motor_sub =
      nh_.subscribe<motor_msg::MotorStamped>("motor/state", 1000,
                                             motor_feedback_cb);
  core::Subscriber<force_msg::LegForceStamped> &force_sub =
      nh_.subscribe<force_msg::LegForceStamped>("force/force_state", 1000,
                                                force_feedback_cb);
  core::Subscriber<robot_msg::State> &robot_state_sub =
      nh_.subscribe<robot_msg::State>("robot/state", 1000,
                                      robot_state_feedback_cb);
                                                
  core::Publisher<force_msg::LegForceStamped> &force_pub =
      nh_.advertise<force_msg::LegForceStamped>("force/force_command");

      
  float count = 0.0;
  while (ros::ok()) {
    core::spinOnce();
    ros::spinOnce();

    if (force_ros_msg_updated){
      force_msg::LegForceStamped force_cmd_msg;
      force_msg::LegForce force_A_LF;
      force_msg::LegForce force_B_RF;
      force_msg::LegForce force_C_RH;
      force_msg::LegForce force_D_LH;
      
      mtx.lock();
      force_A_LF.set_pose_x(force_ros_msg.A_LF.force.pose_x);
      force_A_LF.set_pose_y(force_ros_msg.A_LF.force.pose_y);
      force_A_LF.set_force_x(force_ros_msg.A_LF.force.force_x);
      force_A_LF.set_force_y(force_ros_msg.A_LF.force.force_y);
      force_B_RF.set_pose_x(force_ros_msg.B_RF.force.pose_x);
      force_B_RF.set_pose_y(force_ros_msg.B_RF.force.pose_y);
      force_B_RF.set_force_x(force_ros_msg.B_RF.force.force_x);
      force_B_RF.set_force_y(force_ros_msg.B_RF.force.force_y);
      force_C_RH.set_pose_x(force_ros_msg.C_RH.force.pose_x);
      force_C_RH.set_pose_y(force_ros_msg.C_RH.force.pose_y);
      force_C_RH.set_force_x(force_ros_msg.C_RH.force.force_x);
      force_C_RH.set_force_y(force_ros_msg.C_RH.force.force_y);
      force_D_LH.set_pose_x(force_ros_msg.D_LH.force.pose_x);
      force_D_LH.set_pose_y(force_ros_msg.D_LH.force.pose_y);
      force_D_LH.set_force_x(force_ros_msg.D_LH.force.force_x);
      force_D_LH.set_force_y(force_ros_msg.D_LH.force.force_y);
      force_cmd_msg.add_force()->CopyFrom(force_A_LF);
      force_cmd_msg.add_force()->CopyFrom(force_B_RF);
      force_cmd_msg.add_force()->CopyFrom(force_C_RH);
      force_cmd_msg.add_force()->CopyFrom(force_D_LH);

      mtx.unlock();

      force_pub.publish(force_cmd_msg);
      force_ros_msg_updated = 0;
    }

    /*
    if (motor_msg_updated && motor_fb_msg.motors().size() == 8) {
      mtx.lock();
      corgi_ros_bridge::RobotStamped robot_msg;
      robot_msg.A_LF.motor_r.angle = motor_fb_msg.motors(0).angle();
      robot_msg.A_LF.motor_r.torque = motor_fb_msg.motors(0).torque();
      robot_msg.A_LF.motor_r.twist = motor_fb_msg.motors(0).twist();
      robot_msg.A_LF.motor_r.kp = motor_fb_msg.motors(0).kp();
      robot_msg.A_LF.motor_r.ki = motor_fb_msg.motors(0).ki();
      robot_msg.A_LF.motor_r.kd = motor_fb_msg.motors(0).kd();
      robot_msg.A_LF.motor_l.angle = motor_fb_msg.motors(1).angle();
      robot_msg.A_LF.motor_l.torque = motor_fb_msg.motors(1).torque();
      robot_msg.A_LF.motor_l.twist = motor_fb_msg.motors(1).twist();
      robot_msg.A_LF.motor_l.kp = motor_fb_msg.motors(1).kp();
      robot_msg.A_LF.motor_l.ki = motor_fb_msg.motors(1).ki();
      robot_msg.A_LF.motor_l.kd = motor_fb_msg.motors(1).kd();

      robot_msg.B_RF.motor_r.angle = motor_fb_msg.motors(2).angle();
      robot_msg.B_RF.motor_r.torque = motor_fb_msg.motors(2).torque();
      robot_msg.B_RF.motor_r.twist = motor_fb_msg.motors(2).twist();
      robot_msg.B_RF.motor_r.kp = motor_fb_msg.motors(2).kp();
      robot_msg.B_RF.motor_r.ki = motor_fb_msg.motors(2).ki();
      robot_msg.B_RF.motor_r.kd = motor_fb_msg.motors(2).kd();
      robot_msg.B_RF.motor_l.angle = motor_fb_msg.motors(3).angle();
      robot_msg.B_RF.motor_l.torque = motor_fb_msg.motors(3).torque();
      robot_msg.B_RF.motor_l.twist = motor_fb_msg.motors(3).twist();
      robot_msg.B_RF.motor_l.kp = motor_fb_msg.motors(3).kp();
      robot_msg.B_RF.motor_l.ki = motor_fb_msg.motors(3).ki();
      robot_msg.B_RF.motor_l.kd = motor_fb_msg.motors(3).kd();

      robot_msg.C_RH.motor_r.angle = motor_fb_msg.motors(4).angle();
      robot_msg.C_RH.motor_r.torque = motor_fb_msg.motors(4).torque();
      robot_msg.C_RH.motor_r.twist = motor_fb_msg.motors(4).twist();
      robot_msg.C_RH.motor_r.kp = motor_fb_msg.motors(4).kp();
      robot_msg.C_RH.motor_r.ki = motor_fb_msg.motors(4).ki();
      robot_msg.C_RH.motor_r.kd = motor_fb_msg.motors(4).kd();
      robot_msg.C_RH.motor_l.angle = motor_fb_msg.motors(5).angle();
      robot_msg.C_RH.motor_l.torque = motor_fb_msg.motors(5).torque();
      robot_msg.C_RH.motor_l.twist = motor_fb_msg.motors(5).twist();
      robot_msg.C_RH.motor_l.kp = motor_fb_msg.motors(5).kp();
      robot_msg.C_RH.motor_l.ki = motor_fb_msg.motors(5).ki();
      robot_msg.C_RH.motor_l.kd = motor_fb_msg.motors(5).kd();

      robot_msg.D_LH.motor_r.angle = motor_fb_msg.motors(6).angle();
      robot_msg.D_LH.motor_r.torque = motor_fb_msg.motors(6).torque();
      robot_msg.D_LH.motor_r.twist = motor_fb_msg.motors(6).twist();
      robot_msg.D_LH.motor_r.kp = motor_fb_msg.motors(6).kp();
      robot_msg.D_LH.motor_r.ki = motor_fb_msg.motors(6).ki();
      robot_msg.D_LH.motor_r.kd = motor_fb_msg.motors(6).kd();
      robot_msg.D_LH.motor_l.angle = motor_fb_msg.motors(7).angle();
      robot_msg.D_LH.motor_l.torque = motor_fb_msg.motors(7).torque();
      robot_msg.D_LH.motor_l.twist = motor_fb_msg.motors(7).twist();
      robot_msg.D_LH.motor_l.kp = motor_fb_msg.motors(7).kp();
      robot_msg.D_LH.motor_l.ki = motor_fb_msg.motors(7).ki();
      robot_msg.D_LH.motor_l.kd = motor_fb_msg.motors(7).kd();

      if (force_fb_msg.force().size() == 4) {
        std::cout << force_fb_msg.force(1).force_y() << std::endl;
        robot_msg.A_LF.force.force_x = force_fb_msg.force(0).force_x();
        robot_msg.A_LF.force.force_y = force_fb_msg.force(0).force_y();

        robot_msg.B_RF.force.force_x = force_fb_msg.force(1).force_x();
        robot_msg.B_RF.force.force_y = force_fb_msg.force(1).force_y();

        robot_msg.C_RH.force.force_x = force_fb_msg.force(2).force_x();
        robot_msg.C_RH.force.force_y = force_fb_msg.force(2).force_y();

        robot_msg.D_LH.force.force_x = force_fb_msg.force(3).force_x();
        robot_msg.D_LH.force.force_y = force_fb_msg.force(3).force_y();
      }

      if (force_fb_msg.impedance().size() == 4) {
        robot_msg.A_LF.impedance.K0_x = force_fb_msg.impedance(0).k0_x();
        robot_msg.A_LF.impedance.K0_y = force_fb_msg.impedance(0).k0_y();
        robot_msg.B_RF.impedance.K0_x = force_fb_msg.impedance(1).k0_x();
        robot_msg.B_RF.impedance.K0_y = force_fb_msg.impedance(1).k0_y();
        robot_msg.C_RH.impedance.K0_x = force_fb_msg.impedance(2).k0_x();
        robot_msg.C_RH.impedance.K0_y = force_fb_msg.impedance(2).k0_y();
        robot_msg.D_LH.impedance.K0_x = force_fb_msg.impedance(3).k0_x();
        robot_msg.D_LH.impedance.K0_y = force_fb_msg.impedance(3).k0_y();
      }

      mtx.unlock();

      ros_legpub_.publish(robot_msg);
      force_msg_updated = 0;
      motor_msg_updated = 0;
    }
    */


    if (robot_state_msg_update){
      mtx.lock();
      corgi_ros_bridge::RobotStamped robot_msg;
      robot_msg.pose.position.x = robot_state_fb_msg.pose().position().x();
      robot_msg.pose.position.y = robot_state_fb_msg.pose().position().y();
      robot_msg.pose.position.z = robot_state_fb_msg.pose().position().z();
      robot_msg.pose.orientation.x = robot_state_fb_msg.pose().orientation().x();
      robot_msg.pose.orientation.y = robot_state_fb_msg.pose().orientation().y();
      robot_msg.pose.orientation.z = robot_state_fb_msg.pose().orientation().z();
      robot_msg.pose.orientation.w = robot_state_fb_msg.pose().orientation().w();
      robot_msg.twist.linear.x = robot_state_fb_msg.twist().linear().x();
      robot_msg.twist.linear.y = robot_state_fb_msg.twist().linear().y();
      robot_msg.twist.linear.z = robot_state_fb_msg.twist().linear().z();
      robot_msg.twist.angular.x = robot_state_fb_msg.twist().angular().x();
      robot_msg.twist.angular.y = robot_state_fb_msg.twist().angular().y();
      robot_msg.twist.angular.z = robot_state_fb_msg.twist().angular().z();

      if (force_msg_updated){
        robot_msg.A_LF.force.force_x = force_fb_msg.force(0).force_x();
        robot_msg.A_LF.force.force_y = force_fb_msg.force(0).force_y();
        robot_msg.B_RF.force.force_x = force_fb_msg.force(1).force_x();
        robot_msg.B_RF.force.force_y = force_fb_msg.force(1).force_y();
        robot_msg.C_RH.force.force_x = force_fb_msg.force(2).force_x();
        robot_msg.C_RH.force.force_y = force_fb_msg.force(2).force_y();
        robot_msg.D_LH.force.force_x = force_fb_msg.force(3).force_x();
        robot_msg.D_LH.force.force_y = force_fb_msg.force(3).force_y();
      }

      mtx.unlock();

      ros_robot_state_pub.publish(robot_msg);
      robot_state_msg_update = 0;
    }

    /*
    corgi_ros_bridge::RobotStamped robot_msg;
    robot_msg.A_LF.motor_r.angle = sin(count);
    robot_msg.A_LF.motor_r.torque = 0;
    robot_msg.A_LF.motor_r.twist = 0;
    robot_msg.A_LF.motor_r.kp = 0;
    robot_msg.A_LF.motor_r.ki = 0;
    robot_msg.A_LF.motor_r.kd = 0;
    robot_msg.A_LF.motor_l.angle = 0;
    robot_msg.A_LF.motor_l.torque = 0;
    robot_msg.A_LF.motor_l.twist = 0;
    robot_msg.A_LF.motor_l.kp = 0;
    robot_msg.A_LF.motor_l.ki = 0;
    robot_msg.A_LF.motor_l.kd = 0;
    ros_legpub_.publish(robot_msg);
    */

    /*
    motor_msg::MotorStamped motor_cmd_msg;
    motor_msg::Motor A_LF_motor_r;
    motor_msg::Motor A_LF_motor_l;
    motor_msg::Motor B_RF_motor_r;
    motor_msg::Motor B_RF_motor_l;
    motor_msg::Motor C_RH_motor_r;
    motor_msg::Motor C_RH_motor_l;
    motor_msg::Motor D_RF_motor_r;
    motor_msg::Motor D_RF_motor_l;
    A_LF_motor_r.set_angle(1);
    A_LF_motor_l.set_angle(1);
    B_RF_motor_r.set_angle(1);
    B_RF_motor_l.set_angle(1);
    C_RH_motor_r.set_angle(1);
    C_RH_motor_l.set_angle(1);
    D_RF_motor_r.set_angle(1);
    D_RF_motor_l.set_angle(1);
    motor_cmd_msg.add_motors()->CopyFrom(A_LF_motor_r);
    motor_cmd_msg.add_motors()->CopyFrom(A_LF_motor_l);
    motor_cmd_msg.add_motors()->CopyFrom(B_RF_motor_r);
    motor_cmd_msg.add_motors()->CopyFrom(B_RF_motor_l);
    motor_cmd_msg.add_motors()->CopyFrom(C_RH_motor_r);
    motor_cmd_msg.add_motors()->CopyFrom(C_RH_motor_l);
    motor_cmd_msg.add_motors()->CopyFrom(D_RF_motor_r);
    motor_cmd_msg.add_motors()->CopyFrom(D_RF_motor_l);
    motor_pub.publish(motor_cmd_msg);
    */

    count += 0.001;
    rt.sleep();
  }

  return 0;
}
