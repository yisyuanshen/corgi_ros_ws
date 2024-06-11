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
        rospy.loginfo(f'ROS Clock: {rospy.get_rostime()}')
        
        motor_cmd_msg = RobotStamped()

        motor_cmd_msg.msg_type = 'motor'
        
        # if count < 200:
        #     theta = 17 + 100 / 200 * count
        #     beta = 0
        # else:
        #     theta = 17 + 50 * (math.cos((count-200)/100*math.pi)+1)
        #     beta = 50 * (-math.cos((count-200)/200*math.pi)+1)
        
        theta = 17 - 40 * (math.cos(count/300*math.pi)-1)
        beta = 0
        
        print(f'TB = ({theta}, {beta})')
        
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
        motor_cmd_msg.A_LF.motor_l.angle = phi_A[1]
        motor_cmd_msg.B_RF.motor_r.angle = phi_B[0]
        motor_cmd_msg.B_RF.motor_l.angle = phi_B[1]
        motor_cmd_msg.C_RH.motor_r.angle = phi_C[0]
        motor_cmd_msg.C_RH.motor_l.angle = phi_C[1]
        motor_cmd_msg.D_LH.motor_r.angle = phi_D[0]
        motor_cmd_msg.D_LH.motor_l.angle = phi_D[1]
        
        motor_cmd_msg.A_LF.motor_r.twist = 0
        motor_cmd_msg.A_LF.motor_r.torque = 0
        motor_cmd_msg.A_LF.motor_r.kp = 90
        motor_cmd_msg.A_LF.motor_r.ki = 0
        motor_cmd_msg.A_LF.motor_r.kd = 1.75
        
        motor_cmd_msg.A_LF.motor_l.twist = 0
        motor_cmd_msg.A_LF.motor_l.torque = 0
        motor_cmd_msg.A_LF.motor_l.kp = 90
        motor_cmd_msg.A_LF.motor_l.ki = 0
        motor_cmd_msg.A_LF.motor_l.kd = 1.75
        
        motor_cmd_msg.B_RF.motor_r.twist = 0
        motor_cmd_msg.B_RF.motor_r.torque = 0
        motor_cmd_msg.B_RF.motor_r.kp = 90
        motor_cmd_msg.B_RF.motor_r.ki = 0
        motor_cmd_msg.B_RF.motor_r.kd = 1.75
        
        motor_cmd_msg.B_RF.motor_l.twist = 0
        motor_cmd_msg.B_RF.motor_l.torque = 0
        motor_cmd_msg.B_RF.motor_l.kp = 90
        motor_cmd_msg.B_RF.motor_l.ki = 0
        motor_cmd_msg.B_RF.motor_l.kd = 1.75
        
        motor_cmd_msg.C_RH.motor_r.twist = 0
        motor_cmd_msg.C_RH.motor_r.torque = 0
        motor_cmd_msg.C_RH.motor_r.kp = 90
        motor_cmd_msg.C_RH.motor_r.ki = 0
        motor_cmd_msg.C_RH.motor_r.kd = 1.75
        
        motor_cmd_msg.C_RH.motor_l.twist = 0
        motor_cmd_msg.C_RH.motor_l.torque = 0
        motor_cmd_msg.C_RH.motor_l.kp = 90
        motor_cmd_msg.C_RH.motor_l.ki = 0
        motor_cmd_msg.C_RH.motor_l.kd = 1.75
        
        motor_cmd_msg.D_LH.motor_r.twist = 0
        motor_cmd_msg.D_LH.motor_r.torque = 0
        motor_cmd_msg.D_LH.motor_r.kp = 90
        motor_cmd_msg.D_LH.motor_r.ki = 0
        motor_cmd_msg.D_LH.motor_r.kd = 1.75
        
        motor_cmd_msg.D_LH.motor_l.twist = 0
        motor_cmd_msg.D_LH.motor_l.torque = 0
        motor_cmd_msg.D_LH.motor_l.kp = 90
        motor_cmd_msg.D_LH.motor_l.ki = 0
        motor_cmd_msg.D_LH.motor_l.kd = 1.75
        
        
        pub.publish(motor_cmd_msg)

        count += 1

        rate.sleep()
                
        
if __name__ == '__main__':
    try:
        publish_motor_data()
    except rospy.ROSInterruptException:
        pass
