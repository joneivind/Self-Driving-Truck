#!/usr/bin/env python

'''
* ROS Waypoint path logger node *******************
 
 Saves and publishes waypoints (with pose) as 
 a ROS path msg.

 By Jon Eivind Stranden @ NTNU 2019

****************************************************
'''

import rospy
import numpy as np
import atexit
import tf
from std_msgs.msg import Int16
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

save_wp = False

path_msg = Path()
path_msg.header.frame_id = "wp_path"
pose_index = 0

def save_waypoint(data):

    global save_wp
    global pose_index
    global path_msg

    if save_wp is True:

        pose_msg = PoseStamped()

        pose_msg.header.frame_id = "path_pose"
        pose_msg.header.seq = pose_index

        pose_msg.pose.position.x = data.pose.pose.position.x
        pose_msg.pose.position.y = data.pose.pose.position.y
        
        pose_msg.pose.orientation.x = data.pose.pose.orientation.x
        pose_msg.pose.orientation.y = data.pose.pose.orientation.y
        pose_msg.pose.orientation.z = data.pose.pose.orientation.z
        pose_msg.pose.orientation.w = data.pose.pose.orientation.w

        path_msg.poses.append(pose_msg)

        pose_index += 1


def joy_callback(data):
    
    global save_wp
    global file
    global path_msg
    global pose_index

    save_button = data.buttons[0]

    if save_button == 1 and save_wp is False:
        save_wp = True
        path_msg.poses = []

    elif save_button == 1 and save_wp is True:
        save_wp = False
        pose_index = 0

def waypoints_logger():

    global save_wp
    global path_msg

    rospy.init_node('waypoints_logger', anonymous=True)

    path_pub = rospy.Publisher('wp_path', Path, queue_size=10)    
    wp_active_pub = rospy.Publisher('wp_logger_active', Int16, queue_size=10)  

    wp_active_msg = Int16()

    #rospy.Subscriber('pf/pose/odom', Odometry, save_waypoint) # For move_base
    rospy.Subscriber('scanmatch_odom', Odometry, save_waypoint)
    rospy.Subscriber("joy", Joy, joy_callback)
    
    rate = rospy.Rate(10) # hz
    
    tf_br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        
        path_msg.header.stamp = rospy.get_rostime() # Only update timestamp

        # Tell others if waypoint logger is active
        if save_wp is True:
            wp_active_msg.data = 1
        else:
            wp_active_msg.data = 0

        path_pub.publish(path_msg)
        wp_active_pub.publish(wp_active_msg)
        
        tf_br.sendTransform((0.0, 0.0, 0.0), quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now() , "wp_path", "map")
        
        rate.sleep()


if __name__ == '__main__':
    try:
        waypoints_logger()
    except rospy.ROSInterruptException:
        pass
