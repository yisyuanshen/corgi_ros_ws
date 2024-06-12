#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math

def publish_force_data():
    rospy.init_node('force_publisher')
    
    pub = rospy.Publisher('robot/command', RobotStamped, queue_size=1)
    
    rate = rospy.Rate(100)
    
    count = 0
    while not rospy.is_shutdown(): 
        rospy.loginfo(f'Force Command Published')
        
        force_cmd_msg = RobotStamped()

        force_cmd_msg.msg_type = 'force'
        
        force_cmd_msg.A_LF.force.pose_x = 0.08 * math.sin(count/800*math.pi)
        force_cmd_msg.A_LF.force.pose_y = -0.15 + 0.04 * math.cos(count/800*math.pi)
        force_cmd_msg.A_LF.force.force_x = 10
        force_cmd_msg.A_LF.force.force_y = 80
        
        force_cmd_msg.B_RF.force.pose_x = 0.08 * math.sin(count/800*math.pi)
        force_cmd_msg.B_RF.force.pose_y = -0.15 + 0.04 * math.cos(count/800*math.pi)
        force_cmd_msg.B_RF.force.force_x = 10
        force_cmd_msg.B_RF.force.force_y = 80
        
        force_cmd_msg.C_RH.force.pose_x = 0.08 * math.sin(count/800*math.pi)
        force_cmd_msg.C_RH.force.pose_y = -0.15 + 0.04 * math.cos(count/800*math.pi)
        force_cmd_msg.C_RH.force.force_x = 10
        force_cmd_msg.C_RH.force.force_y = 80
        
        force_cmd_msg.D_LH.force.pose_x = 0.08 * math.sin(count/800*math.pi)
        force_cmd_msg.D_LH.force.pose_y = -0.15 + 0.04 * math.cos(count/800*math.pi)
        force_cmd_msg.D_LH.force.force_x = 10
        force_cmd_msg.D_LH.force.force_y = 80
        
        pub.publish(force_cmd_msg)

        count += 1
        
        rate.sleep()

        
if __name__ == '__main__':
    try:
        publish_force_data()
    except rospy.ROSInterruptException:
        pass
