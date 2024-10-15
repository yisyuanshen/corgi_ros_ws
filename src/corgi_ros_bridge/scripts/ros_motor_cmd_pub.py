#!/usr/bin/env python3
import rospy
from corgi_msgs.msg import MotorCmdStamped


def publish_motor_cmd():
    rospy.init_node('motor_publisher')
    
    pub = rospy.Publisher('motor/command', MotorCmdStamped, queue_size=10)
    
    rate = rospy.Rate(1000)
    
    motor_cmd = MotorCmdStamped()
    
    loop_counter = 0
    while not rospy.is_shutdown():
        
        eta_cmd = [[30, 30],
                   [60, 60],
                   [90, 90],
                   [120, 120]]
        
        motor_cmd.header.seq = loop_counter
        motor_cmd.header.stamp = rospy.Time.now()
        
        motor_cmd_modules = [motor_cmd.module_a, motor_cmd.module_b,
                             motor_cmd.module_c, motor_cmd.module_d]
        
        for i in range(4):
            motor_cmd_modules[i].theta = eta_cmd[i][0]
            motor_cmd_modules[i].beta  = eta_cmd[i][1]
            motor_cmd_modules[i].kp  = 90
            motor_cmd_modules[i].ki  = 0
            motor_cmd_modules[i].kd  = 1.75
        
        pub.publish(motor_cmd)

        loop_counter += 1

        rate.sleep()
    
    
if __name__ == '__main__':
    try:
        publish_motor_cmd()
    except rospy.ROSInterruptException:
        pass
