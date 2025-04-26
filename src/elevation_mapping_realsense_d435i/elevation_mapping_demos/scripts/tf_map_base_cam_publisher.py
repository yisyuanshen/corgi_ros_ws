#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('tf_publisher_node')
    
    tf_br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(300)  # 设置发布频率为100Hz
    
    while not rospy.is_shutdown():
        # 父base_link到子camera_color_optical_frame的tf变换
        transform_base_to_camera = TransformStamped()
        transform_base_to_camera.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
        transform_base_to_camera.child_frame_id = 'camera_depth_optical_frame'
        # 'camera_color_optical_frame'
        transform_base_to_camera.transform.translation.x = 0.21901592885708546  # 设置平移信息
        transform_base_to_camera.transform.translation.y = -0.03913728928196865  # 设置平移信息
        transform_base_to_camera.transform.translation.z = 0.024992131311648152  # 设置平移信息
        transform_base_to_camera.transform.rotation.w = -0.301152  # 设置旋转信息
        transform_base_to_camera.transform.rotation.x = 0.641753  # 设置旋转信息
        transform_base_to_camera.transform.rotation.y = -0.638628  # 设置旋转信息
        transform_base_to_camera.transform.rotation.z = 0.299357  # 设置旋转信息
        
        # 父map到父base_link的tf变换
        transform_map_to_base = TransformStamped()
        transform_map_to_base.header = Header(stamp=rospy.Time.now(), frame_id='map')
        transform_map_to_base.child_frame_id = 'base_link'
        transform_map_to_base.transform.translation.x = 0.1  # 设置平移信息
        transform_map_to_base.transform.translation.y = 0.0  # 设置平移信息
        transform_map_to_base.transform.translation.z = 0.0  # 设置平移信息
        transform_map_to_base.transform.rotation.w = 1.0  # 设置旋转信息
        transform_map_to_base.transform.rotation.x = 0.0  # 设置旋转信息
        transform_map_to_base.transform.rotation.y = 0.0  # 设置旋转信息
        transform_map_to_base.transform.rotation.z = 0.0  # 设置旋转信息

        # 发布tf变换消息
        tf_br.sendTransform(transform_base_to_camera)
        tf_br.sendTransform(transform_map_to_base)
        
        rate.sleep()
