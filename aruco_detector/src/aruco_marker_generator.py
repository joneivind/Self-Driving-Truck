#!/usr/bin/env python

'''
* ArUco marker generator **************************
 
 For creating new aruco markers. Requires OpenCV.

 By Jon Eivind Stranden @ NTNU 2019

****************************************************
'''

import numpy as np
import cv2
import cv2.aruco as aruco

# Set marker id here (int)
marker_id = 2

# Set size and type of aruco marker, ex 2X2, 5X5
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

# Generate a marker image, last param is filesize
img = aruco.drawMarker(aruco_dict, marker_id, 1700)
cv2.imwrite("marker_id_" + str(marker_id) + ".jpg", img)
cv2.imshow('frame', img)

# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()