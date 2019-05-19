import numpy as np
import cv2
import cv2.aruco as aruco

# Set the id of the marker (int)
generated_marker_id = 2

# Select type of aruco marker (size)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

# Create an image from the marker
# the last param is the total image size
img = aruco.drawMarker(aruco_dict, generated_marker_id, 1700)
cv2.imwrite("marker_id_" + str(generated_marker_id) + ".jpg", img)
cv2.imshow('frame', img)

# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()