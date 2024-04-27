#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math

def publish_motor_data():
    rospy.init_node('motor_publisher', anonymous=True)
    
    pub = rospy.Publisher('robot/command', RobotStamped, queue_size=10)
    
    rate = rospy.Rate(1000)
    
    count = 0
    while not rospy.is_shutdown():
        rospy.loginfo(f'ROS Sim Clock: {rospy.get_rostime()}')
        
        motor_cmd_msg = RobotStamped()

        motor_cmd_msg.msg_type = 'motor'

        motor_cmd_msg.A_LF.motor_r.angle = (math.sin(count/1000*math.pi)+1)/3
        motor_cmd_msg.A_LF.motor_l.angle = (-math.sin(count/1000*math.pi)+1)/3

        motor_cmd_msg.B_RF.motor_r.angle = (math.sin(count/1000*math.pi)+1)/3
        motor_cmd_msg.B_RF.motor_l.angle = (-math.sin(count/1000*math.pi)+1)/3
        
        motor_cmd_msg.C_RH.motor_r.angle = (math.sin(count/1000*math.pi)+1)/3
        motor_cmd_msg.C_RH.motor_l.angle = (-math.sin(count/1000*math.pi)+1)/3
        
        motor_cmd_msg.D_LH.motor_r.angle = (math.sin(count/1000*math.pi)+1)/3
        motor_cmd_msg.D_LH.motor_l.angle = (-math.sin(count/1000*math.pi)+1)/3
            
        pub.publish(motor_cmd_msg)

        count += 1

        rate.sleep()        
        
        
if __name__ == '__main__':
    try:
        publish_motor_data()
    except rospy.ROSInterruptException:
        pass
