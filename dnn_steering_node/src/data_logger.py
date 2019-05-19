#!/usr/bin/env python

# ROS node for logging training data
# By Jon Eivind Stranden 2019

import cv2
from time import gmtime, strftime
import atexit
import rospy
from sensor_msgs.msg import Joy

# Append to log file
file = open('training_data.csv', 'a+') 

joy_steering = 0.0
joy_vel = 0.0


def joy_callback(data):

    # Get joystick steering value

    global joy_steering
    global joy_vel

    joy_steering = data.axes[2]
    joy_vel = data.axes[1]


def shutdown():
    file.close()
    cap = cv2.VideoCapture(1)
    cap.release()
    cv2.destroyAllWindows()
    print('Goodbye!')


def data_logger():

    global file
    global joy_steering
    global joy_vel
    
    cap = cv2.VideoCapture(1) # Select camera device
    cap.set(3, 340) # Set resolution of camera input x,y
    cap.set(4, 180)
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # turn of auto exposure

    # Subscribes to joy topic for steering input from controller
    rospy.Subscriber("joy", Joy, joy_callback)

    rospy.init_node('car_interface', anonymous=True)
    rate = rospy.Rate(20) # 20hz

    i = 0 # img counter

    while not rospy.is_shutdown():

        ret, img = cap.read() # Get frame from cam
        #cv2.imshow('frame', img) # Show current frame

        if joy_vel > 0.0: # Save data only when moving forward

            cv2.imwrite('img/img' + strftime('-%Y-%m-%d-%H-%M-%S-%f',gmtime()) + '.jpg', img) 
            file.write('%s, %f, %f\n' % ('img/img' + strftime('-%Y-%m-%d-%H-%M-%S-%f',gmtime()) + '.jpg',  joy_steering, joy_vel))

            print("images saved: " + str(i) + '\t\tsteering value: ' + str(joy_steering))

            i+=1

        #if cv2.waitKey(1) & 0xFF == ord('q'): # Quit by pressing 'q'
        #    break

        rate.sleep()


if __name__ == '__main__':
    atexit.register(shutdown)
    print('Logging data...')
    try:
        data_logger()
    except rospy.ROSInterruptException:
        pass
