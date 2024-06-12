#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math
import os


def robot_state_callback(msg):
    rospy.loginfo(f"\n\nReceived robot state:\n"
                  f"\nPosition      : ({msg.pose.position.x: 9.6f}, {msg.pose.position.y: 9.6f}, {msg.pose.position.z: 9.6f})"
                  f"\nOrientation   : ({msg.pose.orientation.x: 9.6f}, {msg.pose.orientation.y: 9.6f}, {msg.pose.orientation.z: 9.6f}, {msg.pose.orientation.w: 9.6f})"
                  f"\nTwist Linear  : ({msg.twist.linear.x: 9.6f}, {msg.twist.linear.y: 9.6f}, {msg.twist.linear.z: 9.6f})"
                  f"\nTwist Angular : ({msg.twist.angular.x: 9.6f}, {msg.twist.angular.y: 9.6f}, {msg.twist.angular.z: 9.6f})"
                  f"\n\nLeg A Pose    : ({msg.A_LF.force.pose_x: 9.6f}, {msg.A_LF.force.pose_y: 9.6f})"
                  f"\nLeg A Force   : ({msg.A_LF.force.force_x: 9.6f}, {msg.A_LF.force.force_y: 9.6f})"
                  f"\nLeg A Motor   : ({msg.A_LF.motor_r.angle: 9.6f}, {msg.A_LF.motor_l.angle: 9.6f})"
                  f"\n\nLeg B Pose    : ({msg.B_RF.force.pose_x: 9.6f}, {msg.B_RF.force.pose_y: 9.6f})"
                  f"\nLeg B Force   : ({msg.B_RF.force.force_x: 9.6f}, {msg.B_RF.force.force_y: 9.6f})"
                  f"\nLeg B Motor   : ({msg.B_RF.motor_r.angle: 9.6f}, {msg.B_RF.motor_l.angle: 9.6f})"
                  f"\n\nLeg C Pose    : ({msg.C_RH.force.pose_x: 9.6f}, {msg.C_RH.force.pose_y: 9.6f})"
                  f"\nLeg C Force   : ({msg.C_RH.force.force_x: 9.6f}, {msg.C_RH.force.force_y: 9.6f})"
                  f"\nLeg C Motor   : ({msg.C_RH.motor_r.angle: 9.6f}, {msg.C_RH.motor_l.angle: 9.6f})"
                  f"\n\nLeg D Pose    : ({msg.D_LH.force.pose_x: 9.6f}, {msg.D_LH.force.pose_y: 9.6f})"
                  f"\nLeg D Force   : ({msg.D_LH.force.force_x: 9.6f}, {msg.D_LH.force.force_y: 9.6f})"
                  f"\nLeg D Motor   : ({msg.D_LH.motor_r.angle: 9.6f}, {msg.D_LH.motor_l.angle: 9.6f})"
                  f"\n\n= = = = =\n"
                )
    
def listen_to_robot_state():
    rospy.init_node('robot_state_subscriber')
    rospy.Subscriber('robot/state', RobotStamped, robot_state_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listen_to_robot_state()
    except rospy.ROSInterruptException:
        pass
