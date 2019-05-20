#!/usr/bin/env python

'''
* ROS Velocity broadcaster node ********************
 
 Publishes the current velocity calculated from the
 electronic speed controller msg (sensorless).

 By Jon Eivind Stranden @ NTNU 2019

****************************************************
'''

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

vel_msg = Float64()

def servo_input_callback(data):

    global vel_msg

    # Get velocity value from car controller
    esc_vel_value = data.linear.x

    # Convert velocity value to [m/s]
    if esc_vel_value <= 70:
        current_velocity = 1.2 * (1-esc_vel_value/90)
    elif esc_vel_value > 70 and esc_vel_value < 120:
        current_velocity = 0.0
    else:
        current_velocity = -0.7 * ((esc_vel_value - 90)/90)

    # Publish esimated velocity
    vel_msg.data = round(current_velocity, 2)


def main():

    global vel_msg

    print "Running odom_esc node..."

    vel_pub = rospy.Publisher('velocity_esc', Float64, queue_size=10)
    rospy.Subscriber("car_cmd", Twist, servo_input_callback)
    rospy.init_node('esc_vel_node', anonymous=True)
    rate = rospy.Rate(100) # hz

    while not rospy.is_shutdown():
        vel_pub.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass