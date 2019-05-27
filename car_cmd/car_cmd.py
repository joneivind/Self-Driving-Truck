#!/usr/bin/env python
# -*- coding: utf-8 -*- 

'''
* ROS Car Controller node **************************
 
 Central node for publishing steering and velocity 
 commands to serial node on RC truck.

 By Jon Eivind Stranden @ NTNU 2019

****************************************************
'''

from math import tan, sqrt, atan2
import numpy as np
import os
import rospy
from std_msgs.msg import Float32, Float64, Int16, Int64MultiArray
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from filters import *


### Settings ###############################

wheelbase_length = 0.28 # axle length L in m
max_steering_angle = 0.449260 # rad

# set speed of vehicle (0=max, 90=min, 91-180=reverse)
car_velocity_straight_speed = 70
car_velocity_cornering_speed = 70
car_velocity_charge_speed = 70

Kp = 8.0 # P speed gain, * NOT CURRENTLY USED *

obstacle_detector_enabled = True # obstacle detector enable/disable 
obstacle_detect_dist = 0.6 # meters
obstacle_detect_fov = 1 # degrees field of view



### Variables ##############################

# Joystick variables
joy_steering_angle = 0
joy_vel = 90
gear_selected = 0
dead_switch = False
dead_switch_lock = False

# Driving mode variable
driving_mode = 0
# Modes:
# 0 = manual, use blue button to log a new path
# 1 = SLAM with path tracking to follow logged path
# 2 = AI controller for following marked lanes without logged path

# For calculating SLAM velocity
current_velocity = 0.0
prev_dist_x = 0
prev_dist_y = 0

# Move_base variables
move_base_velocity = 0.0
move_base_steering_angle = 0.0

# Pure_pursuit variables
pp_steering_angle = 0

# AI controller variables
dnn_steering_angle = 0.0

# CV lane keeper
lane_center_offset_cm = 0.0

# Waypoint logger status
wp_logger_active = False

# Charger status
charger_active = False

# Obstacle detector
obstacle_detected = False # detect variable
current_steering_angle = 0.0


def joy_callback(data):

    # Get joystick commands

    global joy_steering_angle
    global joy_vel
    global gear_selected
    global dead_switch
    global dead_switch_lock
    global wp_logger_active
    global max_steering_angle
    global driving_mode

    # Get joystick values
    joy_steering_angle = (data.axes[2]*max_steering_angle) # right stick
    joy_vel = -(data.axes[1]*90) + 90 # left stick
    gear_val = data.axes[5] # arrow buttons
    dead_switch_button = data.buttons[5] # RB button
    dead_switch_lock_button = data.buttons[7] # RT button
    slam_tracker_mode_button = data.buttons[1] # green button
    ai_mode_button = data.buttons[2] # red button
    manual_mode_button = data.buttons[3] # orange button

    # Set dead switch or dead switch lock
    if dead_switch_button is 1:
        dead_switch = True
    elif dead_switch_lock_button is 1:
        dead_switch_lock = not dead_switch_lock
        dead_switch = not dead_switch
    elif dead_switch_lock is False:
        dead_switch = False

    # Setting driving mode from joycontroller
    if manual_mode_button is 1:
        driving_mode = 0
    elif slam_tracker_mode_button is 1 and wp_logger_active is False:
        driving_mode = 1
    elif ai_mode_button is 1 and wp_logger_active is False:
        driving_mode = 2

    # Gear selector
    if gear_val > 0 and gear_selected < 2:
        gear_selected+=1
    elif gear_val < 0 and gear_selected > 0:
        gear_selected-=1


def dnn_controller_callback(data):
    
    # Get steering angle from deep learning controller

    global dnn_steering_angle
    global max_steering_angle

    dnn_steering_angle = (data.angular.z*max_steering_angle)


def slam_vel_callback(data):

    # Get velocity from LiDAR

    global prev_dist_x
    global prev_dist_y
    global current_velocity

    # Get current position
    dist_x = round(data.pose.position.x, 3)
    dist_y = round(data.pose.position.y, 3)

    # Calculate current speed from SLAM odometry
    current_velocity = sqrt( (dist_x - prev_dist_x)**2 + (dist_y - prev_dist_y)**2 ) / 0.1 # [m/s]

    prev_dist_x = dist_x
    prev_dist_y = dist_y


def move_base_callback(data):

    # Velocity and steering commands from move_base (ROS navi stack)
    
    global move_base_velocity
    global move_base_steering_angle
    global wheelbase_length

    move_base_velocity = data.linear.x
    move_base_steering_angle = atan2(wheelbase_length * data.angular.z, move_base_velocity)


def pure_pursuit_callback(data):

    # Pure Pursuit steering angle

    global pp_steering_angle
    pp_steering_angle = data.angular.z


def lane_center_offset_callback(data):

    # OpenCV lane detector center offset [cm]

    global lane_center_offset_cm
    lane_center_offset_cm = data.data


def wp_logger_status_callback(data):

    # Status of waypoint (path) logger

    global wp_logger_active
    
    wp_status = data.data
    
    if wp_status is 1:
        wp_logger_active = True
    else:
        wp_logger_active = False


def charger_status_callback(data):

    # Status of wireless charger

    global charger_active
    
    charger_status = data.data
    
    if charger_status is 1:
        charger_active = True
    else:
        charger_active = False


def obstacle_callback(data):

    # Obstacle detector using LiDAR
    # Detection area changes with steering angle

    global obstacle_detector_enabled
    global obstacle_detect_dist
    global obstacle_detect_fov
    global obstacle_detected
    global current_steering_angle

    num_detected = 0
    
    steering_offset = int((current_steering_angle-90)/5)

    ranges = np.array([])

    ranges = np.append(ranges, data.ranges[0:19-steering_offset])
    ranges = np.append(ranges, data.ranges[339-steering_offset:359])
    
    for r in ranges:
        if r < obstacle_detect_dist:
            num_detected += 1

    if obstacle_detector_enabled is True:
        # Trigger if more than n degrees field of view is obstructed
        if num_detected >= obstacle_detect_fov:
            obstacle_detected = True
        else:
            obstacle_detected = False


def can_bus_callback(data):

    # Charger status from CAN bus

    global charger_active

    if data.data[1] > 0:
        charger_active = True
    else:
        charger_active = False


def steering_angle_to_servo(angle):

    # Convert radian angle to valid servo output

    global max_steering_angle

    servo_output = -(angle/max_steering_angle)*90 + 90

    return servo_output


def gear_selector_to_servo(gear):

    # Gear select to valid servo output

    if gear is 2:
        gear_cmd = 180.0 # Top gear
    elif gear is 1:
        gear_cmd = 90.0 # Mid gear
    else:
        gear_cmd = 0.0 # Low gear

    return gear_cmd


def PIDControl(target, current):

    # P-controller (for now)

    global Kp

    a = Kp * (target - current)

    return a + 90


def controller():

    global joy_steering_angle
    global dnn_steering_angle
    global pp_steering_angle
    global move_base_steering_angle
    global joy_vel
    global move_base_velocity
    global current_velocity
    global current_steering_angle
    global gear_selected
    global dead_switch
    global wp_logger_active
    global charger_active
    global lane_center_offset_cm
    global obstacle_detected
    global sliding_window_median
    global car_velocity_straight_speed
    global car_velocity_cornering_speed
    global car_velocity_charge_speed
    global driving_mode

    # For reverse hack
    set_reverse = False

    # Set up node
    rospy.init_node('car_interface', anonymous=True)

    # Publisher
    controller_pub = rospy.Publisher('car_cmd', Twist, queue_size=10)
    vel_pub = rospy.Publisher('velocity_real', Float64, queue_size=10)
    led_pub = rospy.Publisher('led_mode', Int16, queue_size=10)    

    controller_msg = Twist()
    vel_msg = Float64()
    led_msg = Int16()

    # Subscriber
    rospy.Subscriber("joy", Joy, joy_callback)
    #rospy.Subscriber("pf/viz/inferred_pose", PoseStamped, slam_vel_callback) # For move_base
    rospy.Subscriber("slam_out_pose", PoseStamped, slam_vel_callback)    
    rospy.Subscriber("cmd_vel", Twist, move_base_callback)
    rospy.Subscriber("pure_pursuit", Twist, pure_pursuit_callback)
    rospy.Subscriber("car_vision/lane_center_offset_cm", Float32, lane_center_offset_callback)
    rospy.Subscriber("dnn_controller", Twist, dnn_controller_callback)
    rospy.Subscriber("wp_logger_active", Int16, wp_logger_status_callback)
    rospy.Subscriber("aruco_charger_status", Int16, charger_status_callback)
    rospy.Subscriber("scan", LaserScan, obstacle_callback)
    rospy.Subscriber("CAN_bus", Int64MultiArray, can_bus_callback) 

    rate = rospy.Rate(10) # hz

    # LED modes:
    led_off = 0
    led_green = 1
    led_green_blink = 2
    led_blue = 3
    led_blue_blink = 4
    led_red = 5
    led_red_blink = 6

    while not rospy.is_shutdown():        

        # Manual control / logging mode (use blue button), joycontroller
        if driving_mode is 0:

            # Update status LED on car
            if wp_logger_active is True:
                led_msg.data = led_blue
            elif wp_logger_active is False:
                if charger_active is True:
                    led_msg.data = led_red
                else:
                    led_msg.data = led_green

            # Set velocity and steering commands for manual mode
            if dead_switch is True:
                controller_msg.linear.x = joy_vel
                controller_msg.angular.z = steering_angle_to_servo(joy_steering_angle)
            else:
                controller_msg.linear.x = 90
                controller_msg.angular.z = 90

            # Set gear select
            controller_msg.linear.z = gear_selector_to_servo(gear_selected)



        # SLAM with Pure pursuit controller
        elif driving_mode is 1 and wp_logger_active is False:

            # Update status LED on car
            if charger_active is True:
                led_msg.data = led_red_blink
            else:
                led_msg.data = led_blue_blink

            # Set velocity and steering commands for autonomous mode
            if dead_switch is True and obstacle_detected is False:

                # Dynamic speed control
                if abs(pp_steering_angle) > 0.1:
                    # used this when cornering
                    controller_msg.linear.x = car_velocity_cornering_speed

                elif charger_active:
                    # use this when charger is active
                    controller_msg.linear.x = car_velocity_charge_speed
                
                else: 
                    # use this when going strait
                    controller_msg.linear.x = car_velocity_straight_speed

                # Steering control
                controller_msg.angular.z = steering_angle_to_servo(pp_steering_angle)

            else:
                controller_msg.linear.x = 90
                #controller_msg.angular.z = 90
            
            # Set gear select
            controller_msg.linear.z = gear_selector_to_servo(gear_selected)



        # AI controller
        elif driving_mode is 2 and wp_logger_active is False:
            
            # Set status led
            led_msg.data = led_green_blink

            if dead_switch is True and obstacle_detected is False:

                # Dynamic speed control
                if abs(dnn_steering_angle) > 0.1:
                    # used this when cornering
                    controller_msg.linear.x = car_velocity_cornering_speed

                elif charger_active:
                    # use this when charger is active
                    controller_msg.linear.x = car_velocity_charge_speed
                
                else: 
                    # use this when going strait
                    controller_msg.linear.x = car_velocity_straight_speed

                controller_msg.angular.z = steering_angle_to_servo(dnn_steering_angle)

            else:
                controller_msg.linear.x = 90
                #controller_msg.angular.z = 90
            
            controller_msg.linear.z = gear_selector_to_servo(gear_selected)



        # Current real velocity
        vel_msg.data = low_pass_filter_median(round(current_velocity, 2))  
        
        # Store steering angle
        current_steering_angle = controller_msg.angular.z


        '''
        # Computer vision lane detector
        else:
            if dead_switch is True:
                controller_msg.angular.z = PIDControl(0, lane_center_offset_cm)
            else:
                controller_msg.linear.x = 90
                controller_msg.angular.z = 90
            
            controller_msg.linear.z = gear_selector_to_servo(gear_selected)  
        '''

        
        '''
        # Move base (for ROS navigation stack)
        else:
            if dead_switch is True:

                if move_base_velocity > 0.0:
                    controller_msg.linear.x = 70 - (50*move_base_velocity)
                    controller_msg.angular.z = steering_angle_to_servo(move_base_steering_angle)
                    if set_reverse is True:
                        set_reverse = False

                elif move_base_velocity < 0.0:
                    # Hack for setting the car in reverse
                    if set_reverse is False:
                        set_reverse = True        
                        for i in range(2):                
                            controller_msg.linear.x = 155
                            car_pub.publish(controller_msg)
                            rate.sleep()
                        for i in range(2):                
                            controller_msg.linear.x = 90
                            car_pub.publish(controller_msg)
                            rate.sleep()
                    else: 
                        controller_msg.linear.x = 155
                    
                    controller_msg.angular.z = -steering_angle_to_servo(move_base_steering_angle)

                else:
                    controller_msg.linear.x = 90
                    controller_msg.angular.z = 90
            else:
                controller_msg.linear.x = 90
            
            controller_msg.linear.z = gear_selector_to_servo(gear_selected)
        '''             
        
        # Publish msgs
        controller_pub.publish(controller_msg)
        vel_pub.publish(vel_msg)
        led_pub.publish(led_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
