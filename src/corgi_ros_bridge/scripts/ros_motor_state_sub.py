#!/usr/bin/env python3
import rospy
from corgi_msgs.msg import MotorStateStamped
import time

def motor_state_callback(msg):
    rospy.loginfo('Motor State Received.')
    rospy.loginfo(f'Header Stamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}')
    rospy.loginfo(' ')
    
def subscribe_motor_state():
    rospy.init_node('motor_subscriber')
    rospy.Subscriber('motor/state', MotorStateStamped, motor_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_motor_state()
    except rospy.ROSInterruptException:
        pass
