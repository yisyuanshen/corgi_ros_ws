#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped

def callback(robot_msg):
    rospy.loginfo("Received robot state:")
    rospy.loginfo("Position: [{}, {}, {}]".format(robot_msg.pose.position.x, robot_msg.pose.position.y, robot_msg.pose.position.z))
    rospy.loginfo("Orientation: [{}, {}, {}, {}]".format(robot_msg.pose.orientation.x, robot_msg.pose.orientation.y, robot_msg.pose.orientation.z, robot_msg.pose.orientation.w))
    rospy.loginfo("Linear Velocity: [{}, {}, {}]".format(robot_msg.twist.linear.x, robot_msg.twist.linear.y, robot_msg.twist.linear.z))
    rospy.loginfo("Angular Velocity: [{}, {}, {}]".format(robot_msg.twist.angular.x, robot_msg.twist.angular.y, robot_msg.twist.angular.z))
    rospy.loginfo("Mod_A Force Estimated: [{}, {}]".format(robot_msg.A_LF.force.force_x, robot_msg.A_LF.force.force_y))
    rospy.loginfo("Mod_B Force Estimated: [{}, {}]".format(robot_msg.B_RF.force.force_x, robot_msg.B_RF.force.force_y))
    rospy.loginfo("Mod_C Force Estimated: [{}, {}]".format(robot_msg.C_RH.force.force_x, robot_msg.C_RH.force.force_y))
    rospy.loginfo("Mod_D Force Estimated: [{}, {}]".format(robot_msg.D_LH.force.force_x, robot_msg.D_LH.force.force_y))
    rospy.loginfo(" ")

def listener():
    rospy.init_node('robot_state_subscriber', anonymous=True)
    rospy.Subscriber("robot/state", RobotStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
