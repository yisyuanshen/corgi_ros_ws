#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import math

def publish_force_data():
    rospy.init_node('force_publisher')
    
    pub = rospy.Publisher('robot/command', RobotStamped, queue_size=10)
    
    rate = rospy.Rate(100)
    
    count = 0
    while not rospy.is_shutdown(): 
        rospy.loginfo(f'ROS Clock: {rospy.get_rostime()}')
        
        force_cmd_msg = RobotStamped()

        force_cmd_msg.msg_type = 'force'
        
        force_cmd_msg.A_LF.force.pose_x = 0.08 * math.sin(count/200*math.pi)
        force_cmd_msg.A_LF.force.pose_y = -0.15 + 0.04 * math.cos(count/200*math.pi)
        force_cmd_msg.A_LF.force.force_x = 10
        force_cmd_msg.A_LF.force.force_y = 80
        
        force_cmd_msg.B_RF.force.pose_x = 0.08 * math.sin(count/200*math.pi)
        force_cmd_msg.B_RF.force.pose_y = -0.15 + 0.04 * math.cos(count/200*math.pi)
        force_cmd_msg.B_RF.force.force_x = 10
        force_cmd_msg.B_RF.force.force_y = 80
        
        force_cmd_msg.C_RH.force.pose_x = 0.08 * math.sin(count/200*math.pi)
        force_cmd_msg.C_RH.force.pose_y = -0.15 + 0.04 * math.cos(count/200*math.pi)
        force_cmd_msg.C_RH.force.force_x = 10
        force_cmd_msg.C_RH.force.force_y = 80
        
        force_cmd_msg.D_LH.force.pose_x = 0.08 * math.sin(count/200*math.pi)
        force_cmd_msg.D_LH.force.pose_y = -0.15 + 0.04 * math.cos(count/200*math.pi)
        force_cmd_msg.D_LH.force.force_x = 10
        force_cmd_msg.D_LH.force.force_y = 80
        
        
        force_cmd_msg.A_LF.impedance.M_x = 0.652
        force_cmd_msg.A_LF.impedance.M_y = 0.652
        force_cmd_msg.A_LF.impedance.K0_x = 20000
        force_cmd_msg.A_LF.impedance.K0_y = 20000
        force_cmd_msg.A_LF.impedance.D_x = 400
        force_cmd_msg.A_LF.impedance.D_y = 600
        force_cmd_msg.A_LF.impedance.adaptive_kp_x = 0  # 1000
        force_cmd_msg.A_LF.impedance.adaptive_kp_y = 0  # 3000
        force_cmd_msg.A_LF.impedance.adaptive_ki_x = 0  # 0
        force_cmd_msg.A_LF.impedance.adaptive_ki_y = 0  # 1800
        force_cmd_msg.A_LF.impedance.adaptive_kd_x = 0  # 50
        force_cmd_msg.A_LF.impedance.adaptive_kd_y = 0  # 50
        
        force_cmd_msg.B_RF.impedance.M_x = 0.652
        force_cmd_msg.B_RF.impedance.M_y = 0.652
        force_cmd_msg.B_RF.impedance.K0_x = 20000
        force_cmd_msg.B_RF.impedance.K0_y = 20000
        force_cmd_msg.B_RF.impedance.D_x = 400
        force_cmd_msg.B_RF.impedance.D_y = 600
        force_cmd_msg.B_RF.impedance.adaptive_kp_x = 0  # 1000
        force_cmd_msg.B_RF.impedance.adaptive_kp_y = 0  # 3000
        force_cmd_msg.B_RF.impedance.adaptive_ki_x = 0  # 0
        force_cmd_msg.B_RF.impedance.adaptive_ki_y = 0  # 1800
        force_cmd_msg.B_RF.impedance.adaptive_kd_x = 0  # 50
        force_cmd_msg.B_RF.impedance.adaptive_kd_y = 0  # 50
        
        force_cmd_msg.C_RH.impedance.M_x = 0.652
        force_cmd_msg.C_RH.impedance.M_y = 0.652
        force_cmd_msg.C_RH.impedance.K0_x = 20000
        force_cmd_msg.C_RH.impedance.K0_y = 20000
        force_cmd_msg.C_RH.impedance.D_x = 400
        force_cmd_msg.C_RH.impedance.D_y = 600
        force_cmd_msg.C_RH.impedance.adaptive_kp_x = 0  # 1000
        force_cmd_msg.C_RH.impedance.adaptive_kp_y = 0  # 3000
        force_cmd_msg.C_RH.impedance.adaptive_ki_x = 0  # 0
        force_cmd_msg.C_RH.impedance.adaptive_ki_y = 0  # 1800
        force_cmd_msg.C_RH.impedance.adaptive_kd_x = 0  # 50
        force_cmd_msg.C_RH.impedance.adaptive_kd_y = 0  # 50
        
        force_cmd_msg.D_LH.impedance.M_x = 0.652
        force_cmd_msg.D_LH.impedance.M_y = 0.652
        force_cmd_msg.D_LH.impedance.K0_x = 20000
        force_cmd_msg.D_LH.impedance.K0_y = 20000
        force_cmd_msg.D_LH.impedance.D_x = 400
        force_cmd_msg.D_LH.impedance.D_y = 600
        force_cmd_msg.D_LH.impedance.adaptive_kp_x = 0  # 1000
        force_cmd_msg.D_LH.impedance.adaptive_kp_y = 0  # 3000
        force_cmd_msg.D_LH.impedance.adaptive_ki_x = 0  # 0
        force_cmd_msg.D_LH.impedance.adaptive_ki_y = 0  # 1800
        force_cmd_msg.D_LH.impedance.adaptive_kd_x = 0  # 50
        force_cmd_msg.D_LH.impedance.adaptive_kd_y = 0  # 50
        
        
        pub.publish(force_cmd_msg)

        count += 1
        
        rate.sleep()

        
if __name__ == '__main__':
    try:
        publish_force_data()
    except rospy.ROSInterruptException:
        pass
