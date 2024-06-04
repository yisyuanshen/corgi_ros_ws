#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import matplotlib.pyplot as plt
import os

# Lists to store time, x, and z data
times = []
x_positions = []
z_positions = []

def robot_state_callback(msg):
    current_time = rospy.get_time()  # Get the current time in seconds
    times.append(current_time)
    x_positions.append(msg.pose.position.x)
    z_positions.append(msg.pose.position.z)

    # Log information, you might want to reduce or format this depending on verbosity required
    rospy.loginfo("Time: {}, x: {}, z: {}".format(current_time, msg.pose.position.x, msg.pose.position.z))

def plot_positions():
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(times, x_positions, label='x position')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (m)')
    plt.title('X Position over Time')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, z_positions, label='z position')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Position (m)')
    plt.title('Z Position over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

def listen_to_robot_state():
    rospy.init_node('robot_state_subscriber', anonymous=True)
    rospy.Subscriber('robot/state', RobotStamped, robot_state_callback)
    rospy.spin()  # This will block until the node is shutdown

if __name__ == '__main__':
    try:
        listen_to_robot_state()
    except rospy.ROSInterruptException:
        pass
    finally:
        plot_positions()  # Plot the positions after the node has been shutdown
