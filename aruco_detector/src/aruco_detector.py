#!/usr/bin/env python

import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from std_msgs.msg import Int16

def aruco_marker_detector(img):

    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_1000)

    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    
    # Draw outline if marker is found
    if ids is not None:
        for i, corner in zip(ids, corners):
            #print('ID: {}; Corners: {}'.format(i, corner))
            # Draw id on marker
            font = cv2.FONT_HERSHEY_SIMPLEX
            str1 = str('id:') + str(i[0])
            cv2.putText(img, str1, (int(corner[0][0][0]), int(corner[0][0][1])-20), font, .9, (255, 255, 255), 1, cv2.LINE_AA)

        # Outline the detected markers
    img = aruco.drawDetectedMarkers(img, corners, borderColor=(50, 200, 50))

    return ids, corners


def main():

    cap = cv2.VideoCapture(1) # Select camera device
    cap.set(3,640) # Set resolution of camera input x,y
    cap.set(4,360)
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # turn of auto exposure    

    # Camera calibration settings
    mtx = np.array([[1537.673020, 0.000000, 902.827327], [0.000000, 1586.529937, 545.745303], [0.000000, 0.000000, 1.000000]])
    dist = np.array([0.064465, -0.138072, 0.001351, -0.001640, 0.000000])

    charge_sign_marker_id = 1
    charge_stop_sign_marker_id = 2

    charging = False

    rospy.init_node('aruco_detector_node', anonymous=True)
    charging_pub = rospy.Publisher('aruco_charger_status', Int16, queue_size=10)
    rate = rospy.Rate(60) # hz

    charging_msg = Int16()

    while not rospy.is_shutdown():

        ret, cap_img = cap.read() # Get frame from cam

        cap_img = cv2.undistort(cap_img, mtx, dist, None, mtx)
        ids, corners = aruco_marker_detector(cap_img) 
        #cv2.imshow('Current frame', cap_img) # Show current input image
        
        if ids is not None and charge_sign_marker_id in ids:

            marker_index = np.where(ids == charge_sign_marker_id)[0][0]
            marker_corners = corners[marker_index][0].astype(int)
            
            if marker_corners[3][1] > 100 and charging is False:
                #print "charging on"
                charging = True
                charging_msg.data = 1.0

        if ids is not None and charge_stop_sign_marker_id in ids:

            marker_index = np.where(ids == charge_stop_sign_marker_id)[0][0]
            marker_corners = corners[marker_index][0].astype(int)
            
            if marker_corners[3][1] > 100 and charging is True:
                #print "charging off"
                charging = False
                charging_msg.data = 0.0
        
        cv2.waitKey(1)
        #if cv2.waitKey(1) & 0xFF == ord('q'): # Quit by pressing 'q'
        #    break

        charging_pub.publish(charging_msg)
        rate.sleep()

    cap.release()
    #cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
