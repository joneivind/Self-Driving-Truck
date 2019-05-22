#!/usr/bin/env python

'''
* rosbag resampler node ***********************************
 
 Node for resampling a rosbag that has different sampling
 rate on the topics. Outputs a .csv-file sampled at 10 Hz.

 By Jon Eivind Stranden @ NTNU 2019
***********************************************************
'''

import atexit
import rospy
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseStamped

# Append to log file
file = open('rosdata.csv', 'a+') 

pos_x = 0.0
pos_y = 0.0
can_bus_amp = 0.0
can_bus_batt_curr = 0.0
can_bus_batt_volt = 0.0
can_bus_crg_curr = 0.0

def can_callback(data):
    global can_bus_amp
    can_bus_amp = data.data[1]

def can_bc_callback(data):
    global can_bus_batt_curr
    can_bus_batt_curr = data.data[1]

def can_bv_callback(data):
    global can_bus_batt_volt
    can_bus_batt_volt = data.data[1]

def can_cc_callback(data):
    global can_bus_crg_curr
    can_bus_crg_curr = data.data[1]

def pose_callback(data):
    global pos_x
    global pos_y
    pos_x = data.pose.position.x
    pos_y = data.pose.position.y

def shutdown():
    file.close()
    print('Goodbye!')

def resampler():

    global file
    global can_bus_amp
    global can_bus_batt_curr
    global can_bus_batt_volt
    global can_bus_crg_curr
    global pos_x
    global pos_y

    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)
    rospy.Subscriber("/CAN_bus", Int64MultiArray, can_callback)
    rospy.Subscriber("/CAN_bus/battery_current", Int64MultiArray, can_bc_callback)
    rospy.Subscriber("/CAN_bus/battery_voltage", Int64MultiArray, can_bv_callback)
    rospy.Subscriber("/CAN_bus/charge_current", Int64MultiArray, can_cc_callback)

    rospy.init_node('car_interface', anonymous=True)

    rate = rospy.Rate(10) # Hz

    # Write headers to file
    file.write('%s, %s, %s, %s, %s, %s\n' % ('pos_x', 'pos_y', 'can_bus_amp', 'can_bus_batt_curr', 'can_bus_batt_volt', 'can_bus_crg_curr'))

    while not rospy.is_shutdown():
        # Write data to file
        file.write('%f, %f, %f, %f, %f, %f\n' % (pos_x, pos_y, can_bus_amp, can_bus_batt_curr, can_bus_batt_volt, can_bus_crg_curr))
        rate.sleep()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Logging data...')
    try:
        resampler()
    except rospy.ROSInterruptException:
        pass
