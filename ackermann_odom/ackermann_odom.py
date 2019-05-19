#!/usr/bin/env python

# ROS Odometry broadcaster for SINTEF RC Truck with wireless charging
# The node uses a mix of velocity commands from the car controller and angular velocity from an IMU to estimate odometry, without the use of wheel sensors
# By Jon Eivind Stranden @ NTNU 2019

import rospy
from math import tan, cos, sin
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
import tf

# settings
wheelbase_ = 0.28 # length of wheelbase, axle to axle [m]

# variables
esc_speed_val = 90.0 # init value for electronic speed controller (0-180)
steering_servo_val = 90.0 # init value for steering servo (0-180)
yaw_imu = 0.0 # yaw value placeholder
ang_vel_z_imu = 0.0
init_x_pos = 0.0
init_y_pos = 0.0
init_yaw = 0.0
new_init_pose = False


def servo_input_callback(data):
    
    global esc_speed_val
    global steering_servo_val

    # Get speed and steering values from car controller
    esc_speed_val = data.linear.x
    steering_servo_val = data.angular.z


def imu_callback(data):

    global yaw_imu
    global ang_vel_z_imu

    # Convert quaternions from IMU orientation to euler to get yaw
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)    
    yaw_imu = yaw

    ang_vel_z_imu = data.angular_velocity.z


def initpose_callback(data):

    global init_x_pos
    global init_y_pos
    global init_yaw
    global new_init_pose

    init_x_pos = data.pose.pose.position.x
    init_y_pos = data.pose.pose.position.y

    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)   
    init_yaw = yaw

    new_init_pose = True


def main():

    global wheelbase_
    global esc_speed_val
    global steering_servo_val
    global yaw_imu
    global ang_vel_z_imu
    global init_x_pos
    global init_y_pos
    global init_yaw
    global new_init_pose

    print "Running odom_esc node..."

    # Publisher
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    tf_br = tf.TransformBroadcaster()

    # Subscriptions
    rospy.Subscriber("car_cmd", Twist, servo_input_callback)
    rospy.Subscriber("imu_data", Imu, imu_callback)    
    rospy.Subscriber("initialpose", PoseWithCovarianceStamped, initpose_callback)

    odom_msg = Odometry()

    # Set up node
    rospy.init_node('ackermann_odom', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    last_stamp = rospy.Time.now()

    x_ = 0.0
    y_ = 0.0
    yaw_ = 0.0

    while not rospy.is_shutdown():

        if new_init_pose is True:
            # Update init pose
            x_ = init_x_pos
            y_ = init_y_pos
            yaw_ = init_yaw
            new_init_pose = False

        # Convert servo output to [m/s]
        if esc_speed_val <= 70:
            current_speed = 1.2 * (1-esc_speed_val/90)
        elif esc_speed_val > 70 and esc_speed_val < 120:
            current_speed = 0.0
        else:
            current_speed = -0.7 * ((esc_speed_val - 90)/90)

        # Delta time
        dt = rospy.Time.now() - last_stamp

        # Get yaw from angular velocity of IMU
        yaw_ += ang_vel_z_imu * dt.to_sec()

        # Calculate speed in x and y direction
        x_dot = current_speed * cos(yaw_)
        y_dot = current_speed * sin(yaw_)

        # Calculate x and y position
        x_ += x_dot * dt.to_sec()
        y_ += y_dot * dt.to_sec()

        # Save timestamp
        last_stamp = rospy.Time.now()

        # Make odometry msg
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = "base_link"

        # Position and orientation
        odom_msg.pose.pose.position.x = x_
        odom_msg.pose.pose.position.y = y_
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sin(yaw_/2.0)
        odom_msg.pose.pose.orientation.w = cos(yaw_/2.0)

        # Uncertainty in position
        odom_msg.pose.covariance[0] = 0.5 # <x
        odom_msg.pose.covariance[7]  = 0.5 # <y
        odom_msg.pose.covariance[35] = 0.4 # <yaw

        # Velocity
        odom_msg.twist.twist.linear.x = current_speed
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = ang_vel_z_imu
        odom_msg.twist.covariance[0] = 0.5 # <x
        odom_msg.twist.covariance[7]  = 0.5 # <y
        odom_msg.twist.covariance[35] = 0.4 # <yaw

        # Publish msgs
        odom_pub.publish(odom_msg)


        # Odom transform to
        tf_br.sendTransform((x_, y_, 0.0), (0.0, 0.0, sin(yaw_/2.0), cos(yaw_/2.0)), rospy.Time.now() , "base_link", "odom")
	tf_br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now() , "odom", "map")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
