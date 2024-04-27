#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math

def publish_force_data():
    rospy.init_node('force_publisher', anonymous=True)
    
    pub = rospy.Publisher('robot/command', RobotStamped, queue_size=10)
    
    rate = rospy.Rate(1000)
    
    count = 0
    while not rospy.is_shutdown(): 
        rospy.loginfo(f'ROS Sim Clock: {rospy.get_rostime()}')
        
        force_cmd_msg = RobotStamped()

        force_cmd_msg.msg_type = 'force'
        
        force_cmd_msg.A_LF.force.pose_x = 0.08 * math.sin(count/1000*math.pi)
        force_cmd_msg.A_LF.force.pose_y = -0.2 + 0.08 * math.cos(count/1000*math.pi)
        force_cmd_msg.A_LF.force.force_x = 10
        force_cmd_msg.A_LF.force.force_y = 80
        
        force_cmd_msg.B_RF.force.pose_x = 0.08 * math.sin(count/1000*math.pi + math.pi*0.5)
        force_cmd_msg.B_RF.force.pose_y = -0.2 + 0.08 * math.cos(count/1000*math.pi + math.pi*0.5)
        force_cmd_msg.B_RF.force.force_x = 10
        force_cmd_msg.B_RF.force.force_y = 80
    
        force_cmd_msg.C_RH.force.pose_x = 0.08 * math.sin(count/1000*math.pi + math.pi)
        force_cmd_msg.C_RH.force.pose_y = -0.2 + 0.08 * math.cos(count/1000*math.pi + math.pi)
        force_cmd_msg.C_RH.force.force_x = 10
        force_cmd_msg.C_RH.force.force_y = 80
    
        force_cmd_msg.D_LH.force.pose_x = 0.08 * math.sin(count/1000*math.pi + math.pi*1.5)
        force_cmd_msg.D_LH.force.pose_y = -0.2 + 0.08 * math.cos(count/1000*math.pi + math.pi*1.5)
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
