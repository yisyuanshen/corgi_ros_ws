#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math
import os


def robot_state_callback(msg):
    rospy.loginfo(f'Received robot state:')
    rospy.loginfo(f'Position({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})')
    rospy.loginfo(f'Orientation({msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z}, {msg.pose.orientation.w})')
    rospy.loginfo(f'Twist Linear({msg.twist.linear.x}, {msg.twist.linear.y}, {msg.twist.linear.z})')
    rospy.loginfo(f'Twist Angular({msg.twist.angular.x}, {msg.twist.angular.y}, {msg.twist.angular.z})')
    rospy.loginfo(f'Leg A Pose({msg.A_LF.force.pose_x}, {msg.A_LF.force.pose_y})')
    rospy.loginfo(f'Leg A Force({msg.A_LF.force.force_x}, {msg.A_LF.force.force_y})')
    rospy.loginfo(f'Leg A Motor({msg.A_LF.motor_r.angle}, {msg.A_LF.motor_l.angle})')
    rospy.loginfo(f'Leg B Pose({msg.B_RF.force.pose_x}, {msg.B_RF.force.pose_y})')
    rospy.loginfo(f'Leg B Force({msg.B_RF.force.force_x}, {msg.B_RF.force.force_y})')
    rospy.loginfo(f'Leg B Motor({msg.B_RF.motor_r.angle}, {msg.B_RF.motor_l.angle})')
    rospy.loginfo(f'Leg C Pose({msg.C_RH.force.pose_x}, {msg.C_RH.force.pose_y})')
    rospy.loginfo(f'Leg C Force({msg.C_RH.force.force_x}, {msg.C_RH.force.force_y})')
    rospy.loginfo(f'Leg C Motor({msg.C_RH.motor_r.angle}, {msg.C_RH.motor_l.angle})')
    rospy.loginfo(f'Leg D Pose({msg.D_LH.force.pose_x}, {msg.D_LH.force.pose_y})')
    rospy.loginfo(f'Leg D Force({msg.D_LH.force.force_x}, {msg.D_LH.force.force_y})')
    rospy.loginfo(f'Leg D Motor({msg.D_LH.motor_r.angle}, {msg.D_LH.motor_l.angle})')
    rospy.loginfo(f'= = = = =')

def listen_to_robot_state():
    rospy.init_node('robot_state_subscriber', anonymous=True)
    rospy.Subscriber('robot/state', RobotStamped, robot_state_callback)
    rospy.spin()
    


if __name__ == '__main__':
    try:
        listen_to_robot_state()
    except rospy.ROSInterruptException:
        pass
