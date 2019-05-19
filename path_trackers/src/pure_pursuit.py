#!/usr/bin/env python

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
Lfc = 0.6  # Look-ahead distance
L = 0.28  # wheel base of vehicle [m]
max_steering_angle = 0.44926 # rad

#################################################

# Path point storage
cx = []
cy = []

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)


def pure_pursuit_control(state, cx, cy):

    ind = calc_target_index(state, cx, cy)

    #if pind >= ind:
    #    ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    # Calc angle alpha between the vehicle heading vector and the look-ahead vector
    alpha = math.atan2(ty - state.y, tx - state.x) - (state.yaw)

    if state.v < 0:  # back
        alpha = math.pi - alpha

    # Dynamic look-ahead distance
    Lf = k * abs(state.v) + Lfc

    # Calc steering wheel angle delta
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    # Cap max steering wheel angle
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)

    return delta, ind


def calc_target_index(state, cx, cy):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    # Dynamic look-ahead distance
    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
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
    
    global cx
    global cy

    px = []
    py = []

    for i, pose in enumerate(data.poses):
        px.append(data.poses[i].pose.position.x)
        py.append(data.poses[i].pose.position.y)

    cx = px
    cy = py


def vel_callback(data):

    global state
    state.v = data.data


def main():

    global state
    global cx
    global cy

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
        if len(cx) != 0:
            steering_angle, target_ind = pure_pursuit_control(state, cx, cy)
            if target_ind is not None:
                tf_br.sendTransform((cx[target_ind], cy[target_ind], 0.0), quaternion_from_euler(0.0, 0.0, 3.1415), rospy.Time.now() , "look_ahead_point", "wp_path")
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
