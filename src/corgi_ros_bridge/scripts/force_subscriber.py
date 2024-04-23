#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped

def callback(robot_msg):
    rospy.loginfo("Received robot force command:")
    rospy.loginfo("Pose X: {}, Pose Y: {}".format(robot_msg.A_LF.force.pose_x, robot_msg.A_LF.force.pose_y))
    rospy.loginfo("Force X: {}, Force Y: {}".format(robot_msg.A_LF.force.force_x, robot_msg.A_LF.force.force_y))
    rospy.loginfo(" ")

def listener():
    rospy.init_node('force_subscriber', anonymous=True)
    rospy.Subscriber("force/force_command", RobotStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
