#!/usr/bin/env python

# Deep learning driving script
# By Jon Eivind Stranden 2019

from keras.models import load_model
from keras.preprocessing.image import img_to_array, load_img
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist


# Load model and set video source
model = load_model('/home/nvidia/nvidia_ws/src/dnn_steering_node/src/model.h5') # Load the network model
video_source = 1 # Set the camera source


def convert_img_to_np_array(img, size=(100,100,3)):

    image = cv2.resize(img, (size[0], size[1])) # Resize image to fit model
    image = image[...,::-1].astype(np.float32) # Convert RGB to BGR
    image = img_to_array(image)
    img_array = np.array([image])

    return img_array


def dnn_controller():

    global model
    global video_source
    global car_vel

    cap = cv2.VideoCapture(video_source) # Select camera device
    cap.set(3, 340) # Set resolution of camera input x,y
    cap.set(4, 180)
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off, does not work on Jetson TX2
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # turn of auto exposure, does not work on Jetson TX2

    # Publisher
    pub = rospy.Publisher('dnn_controller', Twist, queue_size=10)

    # Set up node
    rospy.init_node('dnn_controller_node', anonymous=True)
    rate = rospy.Rate(60) # hz

    while not rospy.is_shutdown():

        ret, cap_img = cap.read() # Get frame from cam

        # Get the predicted steering angle
        predicted_steering = model.predict(convert_img_to_np_array(cap_img))[0][0]|

        #cv2.imshow("Frame", cap_img)
        cv2.waitKey(1)

        # Publish steering msg
        msg = Twist()
        msg.angular.z = predicted_steering
        pub.publish(msg)

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        dnn_controller()
    except rospy.ROSInterruptException:
        pass
