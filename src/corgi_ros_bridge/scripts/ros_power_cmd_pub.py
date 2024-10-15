#!/usr/bin/env python3
import rospy
from corgi_msgs.msg import PowerCmdStamped


def publish_power_cmd():
    rospy.init_node('power_publisher')
    
    pub = rospy.Publisher('power/command', PowerCmdStamped, queue_size=10)
    
    rate = rospy.Rate(1000)
    
    power_cmd = PowerCmdStamped()
    
    loop_counter = 0
    while not rospy.is_shutdown():
        
        power_cmd.header.seq = loop_counter
        power_cmd.header.stamp = rospy.Time.now()
        
        power_cmd.digital = True
        power_cmd.power = True
        power_cmd.motor_mode = 2
        
        pub.publish(power_cmd)

        loop_counter += 1

        rate.sleep()
    
    
if __name__ == '__main__':
    try:
        publish_power_cmd()
    except rospy.ROSInterruptException:
        pass
