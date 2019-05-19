
import numpy as np
import cv2
import cv2.aruco as aruco

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

    cap = cv2.VideoCapture(0) # Select camera device
    cap.set(3,320) # Set resolution of camera input x,y
    cap.set(4,180)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # turn of auto exposure

    stop_sign_marker_id = 1
    go_sign_marker_id = 2

    stop_sign = False    

    while True:

        ret, cap_img = cap.read() # Get frame from cam
        ids, corners = aruco_marker_detector(cap_img) 
        cv2.imshow('Current frame', cap_img) # Show current input image

        if ids is not None and stop_sign_marker_id in ids:

            marker_index = np.where(ids == stop_sign_marker_id)[0][0]
            marker_corners = corners[marker_index][0].astype(int)
            
            if marker_corners[3][1] > 100 and charging is False:
                print "charging on"
                stop_sign = True

        if ids is not None and go_sign_marker_id in ids:

            marker_index = np.where(ids == go_sign_marker_id)[0][0]
            marker_corners = corners[marker_index][0].astype(int)
            
            if marker_corners[3][1] > 100 and charging is True:
                print "charging off"
                charging = False

        if cv2.waitKey(1) & 0xFF == ord('q'): # Quit by pressing 'q'
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()