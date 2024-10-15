#!/usr/bin/env python3
import rospy
from corgi_msgs.msg import PowerStateStamped


def power_state_callback(msg):
    rospy.loginfo('Power State Received.')
    rospy.loginfo(f'Header Stamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}')
    rospy.loginfo(' ')
    
def subscribe_power_state():
    rospy.init_node('power_subscriber')
    rospy.Subscriber('power/state', PowerStateStamped, power_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_power_state()
    except rospy.ROSInterruptException:
        pass
