#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math

def publisher_node():
    rospy.init_node('force_publisher', anonymous=True)
    
    pub = rospy.Publisher('force/force_command', RobotStamped, queue_size=10)
    
    rate = rospy.Rate(500)

    loop_counter = 0
    while not rospy.is_shutdown():
        rospy.loginfo(loop_counter)
        
        robot_msg = RobotStamped()
        robot_msg.A_LF.force.pose_x = -0.2 * math.sin(0.005*loop_counter)
        robot_msg.A_LF.force.pose_y = -0.08 * math.sin(0.015*loop_counter) - 0.2 * math.cos(0.005*loop_counter)
        robot_msg.A_LF.force.force_x = 10
        robot_msg.A_LF.force.force_y = 50
        robot_msg.B_RF.force.pose_x = -0.2 * math.sin(0.005*loop_counter)
        robot_msg.B_RF.force.pose_y = -0.08 * math.sin(0.015*loop_counter) - 0.2 * math.cos(0.005*loop_counter)
        robot_msg.B_RF.force.force_x = 10
        robot_msg.B_RF.force.force_y = 50
        robot_msg.C_RH.force.pose_x = -0.06 * math.sin(0.01*loop_counter)
        robot_msg.C_RH.force.pose_y = -0.19 - 0.06 * math.cos(0.01*loop_counter)
        robot_msg.C_RH.force.force_x = 10
        robot_msg.C_RH.force.force_y = 50
        robot_msg.D_LH.force.pose_x = -0.06 * math.sin(0.01*loop_counter)
        robot_msg.D_LH.force.pose_y = -0.19 - 0.06 * math.cos(0.01*loop_counter)
        robot_msg.D_LH.force.force_x = 10
        robot_msg.D_LH.force.force_y = 50
        
        pub.publish(robot_msg)
        
        rate.sleep()
        
        loop_counter += 1

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass