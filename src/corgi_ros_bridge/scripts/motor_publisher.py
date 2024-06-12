#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math

def tb2phi(theta, beta):
    phi = [theta + beta - math.radians(17),
          -theta + beta + math.radians(17)]
    
    return phi

def publish_motor_data():
    rospy.init_node('motor_publisher')
    
    pub = rospy.Publisher('robot/command', RobotStamped, queue_size=10)
    
    rate = rospy.Rate(100)
    
    count = 0
    while not rospy.is_shutdown():
        rospy.loginfo(f'Motor Command Published')
        
        motor_cmd_msg = RobotStamped()

        motor_cmd_msg.msg_type = 'motor'
        
        
        if count < 1000:
            theta = 17 + 90 * count / 1000
            beta = 0
        
        
        motor_cmd_msg.A_LF.theta = math.radians(theta)
        motor_cmd_msg.A_LF.beta  = math.radians(-beta)
        motor_cmd_msg.B_RF.theta = math.radians(theta)
        motor_cmd_msg.B_RF.beta  = math.radians(beta)
        motor_cmd_msg.C_RH.theta = math.radians(theta)
        motor_cmd_msg.C_RH.beta  = math.radians(beta)
        motor_cmd_msg.D_LH.theta = math.radians(theta)
        motor_cmd_msg.D_LH.beta  = math.radians(-beta)
        
        phi_A = tb2phi(theta=motor_cmd_msg.A_LF.theta, beta=motor_cmd_msg.A_LF.beta)
        phi_B = tb2phi(theta=motor_cmd_msg.B_RF.theta, beta=motor_cmd_msg.B_RF.beta)
        phi_C = tb2phi(theta=motor_cmd_msg.C_RH.theta, beta=motor_cmd_msg.C_RH.beta)
        phi_D = tb2phi(theta=motor_cmd_msg.D_LH.theta, beta=motor_cmd_msg.D_LH.beta)
        
        motor_cmd_msg.A_LF.motor_r.angle = phi_A[0]
        motor_cmd_msg.A_LF.motor_r.torque = 0
        motor_cmd_msg.A_LF.motor_l.angle = phi_A[1]
        motor_cmd_msg.A_LF.motor_l.torque = 0
        motor_cmd_msg.B_RF.motor_r.angle = phi_B[0]
        motor_cmd_msg.B_RF.motor_r.torque = 0
        motor_cmd_msg.B_RF.motor_l.angle = phi_B[1]
        motor_cmd_msg.B_RF.motor_l.torque = 0
        motor_cmd_msg.C_RH.motor_r.angle = phi_C[0]
        motor_cmd_msg.C_RH.motor_r.torque = 0
        motor_cmd_msg.C_RH.motor_l.angle = phi_C[1]
        motor_cmd_msg.C_RH.motor_l.torque = 0
        motor_cmd_msg.D_LH.motor_r.angle = phi_D[0]
        motor_cmd_msg.D_LH.motor_r.torque = 0
        motor_cmd_msg.D_LH.motor_l.angle = phi_D[1]
        motor_cmd_msg.D_LH.motor_l.torque = 0
        
        pub.publish(motor_cmd_msg)

        count += 1

        rate.sleep()
                
        
if __name__ == '__main__':
    try:
        publish_motor_data()
    except rospy.ROSInterruptException:
        pass
