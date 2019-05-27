#!/usr/bin/env python

'''
* ROS Stanley Steering node ************************
 
 Stanley Steering path tracker for following a path.
 
 Stanley Steering algorithm adapted from: 
 https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

 By Jon Eivind Stranden @ NTNU 2019

****************************************************
'''

import math
import os 
import numpy as np
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
import tf


### Tuning settings #################

k = 0.1  # Cross track error gain
wheelbase_length = 0.28  # [m]
max_steering_angle = 0.44926 # [rad]

#####################################

# Path points with yaw storage
course_x = []
course_y = []
course_yaw = []

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)


def stanley_control(state, course_x, course_y, course_yaw):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    current_target_ind, error_front_axle = calc_target_index(state, course_x, course_y)

    # theta_e corrects the heading error
    theta_e = normalize_angle(course_yaw[current_target_ind] - state.yaw)
    
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    
    # Steering control
    delta = theta_e + theta_d

    # Cap max steering wheel angle
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)

    return delta, current_target_ind


def normalize_angle(angle):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, course_x, course_y):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    # Calc front axle position
    front_x = state.x + wheelbase_length * np.cos(state.yaw)
    front_y = state.y + wheelbase_length * np.sin(state.yaw)

    # Search nearest point index
    dx = [front_x - icx for icx in course_x]
    dy = [front_y - icy for icy in course_y]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    error_front_axle = min(d)
    target_idx = d.index(error_front_axle)

    target_yaw = normalize_angle(np.arctan2(
        front_y - course_y[target_idx], front_x - course_x[target_idx]) - state.yaw)
    if target_yaw > 0.0:
        error_front_axle = - error_front_axle

    return target_idx, error_front_axle


def update_state_callback(data):

    global state

    state.x = data.pose.position.x
    state.y = data.pose.position.y

    # Convert quaternions to euler to get yaw
    orientation_list = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    state.yaw = yaw


def path_callback(data):
    
    global course_x
    global course_y
    global course_yaw

    path_x = []
    path_y = []
    path_yaw = []

    for i, pose in enumerate(data.poses):
        path_x.append(data.poses[i].pose.position.x)
        path_y.append(data.poses[i].pose.position.y)
        orientation_list = [data.poses[i].pose.orientation.x, data.poses[i].pose.orientation.y, data.poses[i].pose.orientation.z, data.poses[i].pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        path_yaw.append(yaw)

    course_x = path_x
    course_y = path_y
    course_yaw = path_yaw


def vel_callback(data):

    global state
    state.v = data.data


def main():

    global state
    global course_x
    global course_y
    global course_yaw

    # init node
    rospy.init_node('stanley_controller')
    rate = rospy.Rate(10) # hz

    # Publish
    pub = rospy.Publisher('stanley_steering', Twist, queue_size=10)

    # Get current state of truck
    rospy.Subscriber('slam_out_pose', PoseStamped, update_state_callback, queue_size=10)
    rospy.Subscriber('wp_path', Path, path_callback, queue_size=10)
    rospy.Subscriber('velocity_esc', Float64, vel_callback, queue_size=10)

    # Start a TF broadcaster
    tf_br = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():

        # Get steering angle
        if len(course_x) != 0:
            steering_angle, target_ind = stanley_control(state, course_x, course_y, course_yaw)
            tf_br.sendTransform((course_x[target_ind], course_y[target_ind], 0.0), quaternion_from_euler(0.0, 0.0, 3.1415), rospy.Time.now() , "look_ahead_point", "wp_path")
        else:
            steering_angle = 0.0
            target_ind = 0
        
        # Publish steering msg
        msg = Twist()
        msg.angular.z = steering_angle
        pub.publish(msg)

        rate.sleep()

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
