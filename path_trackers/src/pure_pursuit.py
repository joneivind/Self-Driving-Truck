#!/usr/bin/env python

'''
* ROS Pure Pursuit node ****************************
 
 Pure Pursuit path tracker for following a path.
 
 Pure Pursuit algorithm adapted from: 
 https://github.com/AtsushiSakai/PythonRobotics

 By Jon Eivind Stranden @ NTNU 2019

****************************************************
'''

import rospy
import math
import os 
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Twist, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
import tf


### Tuning settings #############################

k = 0.8  # Look forward gain (Dynamic look-ahead)
look_ahead_dist = 0.6  # Look-ahead distance
wheelbase_length = 0.28  # wheel base of vehicle [m]
max_steering_angle = 0.44926 # rad

#################################################

# Path point storage
course_x = []
course_y = []

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)


def pure_pursuit_control(state, course_x, course_y):

    # Get waypoint target index
    ind = calc_target_index(state, course_x, course_y)

    if ind < len(course_x):
        target_x = course_x[ind]
        target_y = course_y[ind]
    else:
        target_x = course_x[-1]
        target_y = course_y[-1]
        ind = len(course_x) - 1

    # Calc angle alpha between the vehicle heading vector and the look-ahead vector
    alpha = math.atan2(target_y - state.y, target_x - state.x) - (state.yaw)

    if state.v < 0:  # back
        alpha = math.pi - alpha

    # Dynamic look-ahead distance
    dyn_look_ahead_dist = k * abs(state.v) + look_ahead_dist

    # Calc steering wheel angle delta
    delta = math.atan2(2.0 * wheelbase_length * math.sin(alpha) / dyn_look_ahead_dist, 1.0)

    # Cap max steering wheel angle
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)

    return delta, ind


def calc_target_index(state, course_x, course_y):

    # search nearest point index
    dx = [state.x - icx for icx in course_x]
    dy = [state.y - icy for icy in course_y]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    wheelbase_length = 0.0

    # Dynamic look-ahead distance
    dyn_look_ahead_dist = k * state.v + look_ahead_dist

    # search look ahead target point index
    while dyn_look_ahead_dist > wheelbase_length and (ind + 1) < len(course_x):
        dx = course_x[ind + 1] - course_x[ind]
        dy = course_y[ind + 1] - course_y[ind]
        wheelbase_length += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


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

    path_x = []
    path_y = []

    for i, pose in enumerate(data.poses):
        path_x.append(data.poses[i].pose.position.x)
        path_y.append(data.poses[i].pose.position.y)

    course_x = path_x
    course_y = path_y


def vel_callback(data):

    global state
    state.v = data.data


def main():

    global state
    global course_x
    global course_y

    # init node
    rospy.init_node('pure_pursuit')
    rate = rospy.Rate(100) # hz

    # Publish
    pub = rospy.Publisher('pure_pursuit', Twist, queue_size=100)

    # Get current state of truck
    #rospy.Subscriber('pf/viz/inferred_pose', PoseStamped, update_state_callback, queue_size=10)
    rospy.Subscriber('slam_out_pose', PoseStamped, update_state_callback, queue_size=100)
    rospy.Subscriber('wp_path', Path, path_callback, queue_size=10)
    rospy.Subscriber('velocity_esc', Float64, vel_callback, queue_size=10)

    # Start a TF broadcaster
    tf_br = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():

        # Get steering angle
        if len(course_x) != 0:
            steering_angle, target_ind = pure_pursuit_control(state, course_x, course_y)
            if target_ind is not None:
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
