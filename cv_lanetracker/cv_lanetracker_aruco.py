#!/usr/bin/env python

'''
* ROS lane detector node ******************************
 
 ROS/OpenCV lane detector with graphical overlay.
 Publishes the lane center offset [cm] on ROS topic.
 Also contains an ArUco marker detector. 
 ROS functionality and center offset measure added 
 by Jon Eivind Stranden. 

 Most lane detection code courtesy of:
 https://github.com/vamsiramakrishnan/AdvancedLaneLines

 By Jon Eivind Stranden @ NTNU 2019

********************************************************
'''

import cv2
import cv2.aruco as aruco
import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.optimize import curve_fit
from collections import deque
from math import ceil

# ROS
import roslib
roslib.load_manifest('cv_lanetracker_aruco')
import sys
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


warped_size = np.array([640, 480])
original_size = np.array([480, 640])
OFFSET =0

#Define Length of Queue 
queue_len= 10
fCount = 0

# Coefficients Queue 
l_coeff_queue = deque(maxlen=queue_len)
r_coeff_queue = deque(maxlen=queue_len)

# Curvature & Offset Queue 
l_curvature_queue = deque(maxlen=queue_len)
r_curvature_queue = deque(maxlen=queue_len)
l_offset_queue = deque(maxlen=queue_len)
r_offset_queue = deque(maxlen=queue_len)

# Last Mask 
pix_width = 400
ym_per_pix = 0.27/original_size[0] #meters per y pixel
lane_width = np.random.normal(pix_width,250, 20)
xm_per_pix = 0.30/pix_width #meters per x pixel
center_position = original_size[1] * xm_per_pix / 2.

overall_offset = 0

# Flags for Event based triggering of actions 
IsLaneFound=False
isPerspectiveCompute = False
patience = 0

# Display Curvature
disp_left = 0
disp_right =0

# Camera calibration settings
mtx = np.array([[708.4340756775686, 0, 317.9663110540382], [0, 724.3038241696117, 274.0865876256384], [0, 0, 1]])
dist = np.array([0.09702126218642344, -0.1836878268546886, 0.01359685879119158, 0.007942342964235989, 0])

offset_from_lane_center_in_cm = 0.0


def filter_img(image):

    # For filtering out the black parts of the image

    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    #frame = cv2.medianBlur(frame,7)

    kernel = np.ones((5,5),np.float32)/25
    image = cv2.filter2D(image,-1,kernel)
    
    lower_black = np.array([0])
    upper_black = np.array([110])
    
    mask = cv2.inRange(image, lower_black, upper_black)

    background = np.full(image.shape, 255, dtype=np.uint8)

    res = cv2.bitwise_or(background, background, mask=mask)

    return res


def get_lane_center_offset(left_x, right_x, left_y, right_y, img_):
    
    # Get offset in meters from center of lane

    m_per_x = (0.3/img_.shape[1])
    m_per_y = (0.23/img_.shape[0])

    pts_left = np.array([np.flipud(np.transpose(np.vstack([left_x, left_y])))])
    pts_right = np.array([np.transpose(np.vstack([right_x, right_y]))])
    pts = np.hstack((pts_left, pts_right))

    midpoint_x = ((int(pts_right[0][0][0]) - int(pts_left[0][-1][0])) / 2 + int(pts_left[0][-1][0]))
    midpoint_x2 = ((int(pts_right[0][400][0]) - int(pts_left[0][-400][0])) / 2 + int(pts_left[0][-400][0]))

    img_ = cv2.line(img_, (img_.shape[1]/2, 50), (midpoint_x, 50), color=(100, 200, 100), thickness=1, lineType=cv2.LINE_AA)

    #img_ = cv2.circle(img_, (midpoint_x2, 90), 12, color=(100, 200, 100), thickness=-1)

    img_ = cv2.circle(img_, (midpoint_x, 50), 12, color=(100, 200, 100), thickness=-1)
    img_ = cv2.circle(img_, (img_.shape[1]/2, 50), 8, color=(100, 200, 100), thickness=-1)

    error_offset = img_.shape[1]/2 - midpoint_x2

    offset_cm = error_offset * m_per_x * 100

    font = cv2.FONT_HERSHEY_SIMPLEX
    str1 = str(' OFFSET: ') + str(round(offset_cm,2)) + str(' CM')
    cv2.putText(img_, str1, (midpoint_x-120, 120), font, 1, (100, 200, 100), 2, cv2.LINE_AA)

    # Range measure
    for i in range(1,10):
        y_est = round((i*50 * m_per_y)*100 + 2.5, 1)
        img_ = cv2.line(img_, (-10, 480-50*i), (20, 480-50*i), color=(100, 200, 100), thickness=2)
        img_ = cv2.line(img_, (650, 480-50*i), (620, 480-50*i), color=(100, 200, 100), thickness=2)
        str1 = str(y_est)
        cv2.putText(img_, str1, (30, 480-50*i+4), font, 0.4, (100, 200, 100), 1, cv2.LINE_AA)
        cv2.putText(img_, str1, (580, 480-50*i+4), font, 0.4, (100, 200, 100), 1, cv2.LINE_AA)


    return img_, offset_cm


'''

* CITE *********************************************************************

*    Content of some functions has been modified

*    Title: AdvancedLaneLines
*    Author: Vamsi Ramakrishnan
*    Date: Aug 2017
*    Availability: https://github.com/vamsiramakrishnan/AdvancedLaneLines
'''

# Undistort Images 
def undistort_image(mtx_, dist_, img_):
    """
    Undistort the image using distortion coefficients
    :param mtx_: Correction Matrix 
    :param dist_: Distortion Coefficient
    :param img_: Image that needs undistortion
    :return: Distortion Corrected Image
    """
    dst = cv2.undistort(img_, mtx_, dist_, None, mtx_)
    return dst


# Calculate Source and Destination points 
def calc_warp_points():
    """
    :return: Source and Destination pointts 
    """
    src = np.float32 ([
        [162, 479],
        [210, 180],
        [420, 180],
        [480, 479]
    ])

    dst = np.float32 ([
            [172, 530],
            [165, 20],
            [493, 20],
            [470, 530]
        ])
    return src, dst


# Calculate Transform 
def calc_transform(src_, dst_):
    """
    Calculate Perspective and Inverse Perspective Transform Matrices 
    :param src_: Source points
    :param dst_: Destination Points
    :return: Perspective Matrix and Inverse Perspective Transform Matrix
    """
    M_ = cv2.getPerspectiveTransform(src_, dst_)
    Minv_ = cv2.getPerspectiveTransform(dst_, src_)
    return M_, Minv_


# Get perspective transform 
def perspective_transform(M_, img_):
    """

    :param M_: Perspective Matrix 
    :param img_ : Input Image
    :return: Transformed Image 
    """
    img_size = (img_.shape[1],img_.shape[0])
    transformed = cv2.warpPerspective(
        img_,
        M_, img_size,
        flags=cv2.WARP_FILL_OUTLIERS + cv2.INTER_CUBIC)
    return transformed


# Inverse Perspective Transform 
def inv_perspective_transform(Minv_, img_):
    """

    :param M_: Inverse Perspective Transform Matrix
    :param img_: Input Image
    :return: Transformed Image
    """
    img_size = (img_.shape[1], img_.shape[0])
    transformed = cv2.warpPerspective(
        img_,
        Minv_, img_size,
        flags=cv2.WARP_FILL_OUTLIERS + cv2.INTER_CUBIC)
    return transformed


# Extract Pixels from Image 
def extract_pixels(img_):
    """
    Extract all Non Zero Pixels and return X, Y Coordinates
    :param img_: Image from which Non Zero Pixels have to be extracted
    :return: X & Y Coordinates
    """
    non_zero_pixels = np.argwhere(0 < img_)
    x = non_zero_pixels.T[0].astype(np.float32)
    y = non_zero_pixels.T[1].astype(np.float32)
    return x, y


# Get Intercepts 
def get_intercepts(fit, y):
    """
    Get x intercepts for given y value
    :return: 
    :param fit: The polynomial fit
    :param y: Y Coordinates
    :return: X Coordinates
    """
    x = fit[0] * (y * y) + fit[1] * y + fit[2]
    return x


# Draw Polygon based on X, and Y points for Left and Right Lanes on Image
def draw_polygon(left_x, right_x, left_y, right_y, img_):
    """
    Get Left_x, Right_x, Left_y, Right_y, Image , return Image with Polygon
    :return: 
    :param left_x: 
    :param right_x: 
    :param left_y: 
    :param right_y: 
    :param img_: 
    :return: 
    """
    pts_left = np.array([np.flipud(np.transpose(np.vstack([left_x, left_y])))])
    pts_right = np.array([np.transpose(np.vstack([right_x, right_y]))])
    pts = np.hstack((pts_left, pts_right))
    img_ = cv2.polylines(img_, np.int_([pts]), isClosed=False, color=(60, 200, 60), thickness=10, lineType=cv2.LINE_AA)
    img_ = cv2.fillPoly(img_, np.int_(pts), (50, 90, 50))
    return img_


def coordinates_to_imgpts(x, y):
    """
    Convert parameters from X,Y plane to Image Plane Points
    :param x: 
    :param y: 
    :return pts:
    """
    pts = np.array([np.flipud(np.transpose(np.vstack([x, y])))])
    return pts


def draw_polylines(input_img, pts, window_size):
    """
    Draw Polylines for points with given thickness specified by Window Size
    :param input_img: 
    :param pts: 
    :param window_size: 
    :return: Image with Poly Lines 
    """
    return cv2.polylines(input_img, np.int_([pts]), isClosed=False, color=(255, 255, 255),
                         thickness=2 * window_size)


def smoothen_masks(fit, img_, window_size):
    """
     # Use polyfit from the mask points for smoothening them 
    :param fit: 
    :param img_: 
    :param window_size: 
    :return: 
    """
    img_size = img_.shape
    mask_poly = np.zeros_like(img_)
    # Get top to Bottom for refactoring #
    mask_y = np.linspace(0, img_size[0] - 1, img_size[0])
    mask_x = get_intercepts(fit, mask_y)

    # Smoothen the mask #
    pts = coordinates_to_imgpts(mask_x, mask_y)
    mask_poly_smooth = draw_polylines(mask_poly, pts, window_size)
    return mask_poly_smooth


# Use when Lane is Found and Polynomial fit can be used with a Tolerance window to search for lanes
def limited_search(img_, window_size, flag='L'):
    """
    Polynomial search based on previous fit
    :param img_: 
    :param window_size: 
    :param flag: 
    :return: 
    """
    # Initialize Mask with Same Size as Image #
    mask_poly = np.zeros_like(img_)
    # Get previous Coefficients #
    fit = get_last_fit(flag=flag)
    if fit is not None:
        mask_poly_smooth = smoothen_masks(fit, img_, window_size)
        return mask_poly_smooth.astype(np.uint8)
    else:
        return mask_poly
    

# Sliding Window Blind Search to generate a mask for Polynomial fit generation
def blind_search(img_, window_size = 30):
    img_size = img_.shape
    n_segments = 16
    step = img_size[0]//n_segments
    mask_L_poly = np.zeros_like(img_)
    mask_R_poly = np.zeros_like(img_)
    n_steps = 4
    window_start = img_size[1]//2 - 9 * window_size
    window_end = window_start + 6*window_size
    sm = np.sum(img_[img_size[0]-4*step:img_size[0], window_start:window_end], axis=0)
    sm = np.convolve(sm, np.ones((window_size,))/window_size, mode='same')
    argmax = window_start + np.argmax(sm)
    shift = 0
    #plt.figure(figsize=(10,6))
    i =0
    for last in range(img_size[0], 0, -step):
        first_line = max(0, last - n_steps*step)
        sm = np.sum(img_[first_line:last, :], axis=0)
        sm = np.convolve(sm, np.ones((window_size,))/window_size, mode='same')
        window_start = min(max(argmax + int(shift)-window_size//2, 0), img_size[1]-1)
        window_end = min(max(argmax + int(shift) + window_size//2, 0+1), img_size[1])
        new_argmax = window_start + np.argmax(sm[window_start:window_end])
        new_max = np.max(sm[window_start:window_end])
        if new_max <= 2:
            new_argmax = argmax + int(shift)
            shift = shift/2
        if last != img_size[0]:
            shift = shift*0.25 + 0.75*(new_argmax - argmax)
        argmax = new_argmax
        mask_L_poly = cv2.rectangle(mask_L_poly, (argmax-window_size//2, last-step), (argmax+window_size//2, last), 1, thickness=window_size)
        
    not_left = np.logical_not(mask_L_poly).astype(np.uint8)
    filtered_img = cv2.bitwise_and(img_,not_left)
    
    window_start = img_size[1]//2 + 6 * window_size
    window_end = window_start + 6*window_size
    sm = np.sum(filtered_img[img_size[0]-4*step:img_size[0], window_start:window_end], axis=0)
    sm = np.convolve(sm, np.ones((window_size,))/window_size, mode='same')
    argmax = window_start + np.argmax(sm)
    shift = 0
    for last in range(img_size[0], 0, -step):
        first_line = max(0, last - n_steps*step)
        sm = np.sum(filtered_img[first_line:last, :], axis=0)
        sm = np.convolve(sm, np.ones((window_size,))/window_size, mode='same')
        window_start = min(max(argmax + int(shift)-window_size//2, 0), img_size[1]-1)
        window_end = min(max(argmax + int(shift) + window_size//2, 0+1), img_size[1])
        new_argmax = window_start + np.argmax(sm[window_start:window_end])
        new_max = np.max(sm[window_start:window_end])
        if new_max <= 2:
            new_argmax = argmax + int(shift)
            shift = shift/2
        if last != img_size[0]:
            shift = shift*0.25 + 0.75*(new_argmax - argmax)
        argmax = new_argmax
        mask_R_poly = cv2.rectangle(mask_R_poly, (argmax-window_size//2, last-step), (argmax+window_size//2, last), 1, thickness=window_size)

    return mask_L_poly, mask_R_poly


def get_mean_fit(flag='L'):
    """
    Get the mean value of fit "Left" and "Right" based on flag
    :param flag: 
    :return: 
    """
    if flag == 'L':
        return np.mean(np.vstack(l_coeff_queue), axis =0) if len(l_coeff_queue)>1 else l_coeff_queue[-1]
    else:
        return np.mean(np.vstack(r_coeff_queue), axis =0) if len(r_coeff_queue)>1 else r_coeff_queue[-1]

def get_predicted_fit(flag ='L'):
    if flag =='L':
        if len(l_coeff_queue)>1:
            avg_diff_L = np.mean(np.vstack(np.diff(np.vstack(l_coeff_queue), axis=0)), axis =0)
            return np.add(get_mean_fit(flag="L"),avg_diff_L)
        else:
            return l_coeff_queue[-1]
    else:
        if len(r_coeff_queue)>1:
            avg_diff_R = np.mean(np.vstack(np.diff(np.vstack(r_coeff_queue), axis =0)), axis =0)
            return np.add(get_mean_fit(flag="R"),avg_diff_R)
        else:
            return r_coeff_queue[-1]

def get_last_fit(flag='L'):
    """
    Gets the Last Fit depending on the flag 
    :param flag: 
    :return: 
    """
    if flag == 'L':
        return l_coeff_queue[-1]
    else:
        return r_coeff_queue[-1]

def get_mean_curvature(flag='L'):
    if flag =='L':
        return np.mean(l_offset_queue), np.mean(l_curvature_queue)
    else:
        return np.mean(r_offset_queue), np.mean(r_curvature_queue)
                                                
def get_last_curvature(flag='L'):   
    if flag =='L':
        return l_offset_queue[-1], l_curvature_queue[-1]
    else:
        return r_offset_queue[-1], r_curvature_queue[-1]

def return_queue_len(flag='L'):
    if flag =='L':
        return len(l_coeff_queue)
    else:
        return len(r_coeff_queue)

def clear_queues():
    l_coeff_queue.clear()
    r_coeff_queue.clear()
    l_offset_queue.clear()
    l_curvature_queue.clear()
    r_offset_queue.clear()
    r_curvature_queue.clear()
    detected_count =0
    overall_offset =0
    
def pop_queues_left():
    l_coeff_queue.popleft()
    r_coeff_queue.popleft()
    l_curvature_queue.popleft()
    r_offset_queue.popleft()
    r_curvature_queue.popleft()

def append_linecoeffs(fit, flag='L'):
    if flag=='L':
        # left line Coefficients
        l_coeff_queue.append(fit)
    else:
        # Right Line Coefficients
        r_coeff_queue.append(fit)
    return None

def append_curvature(offset, curvature, flag='L'):
    if flag =='L':
        l_curvature_queue.append(curvature)
        l_offset_queue.append(offset)
    else:
        r_curvature_queue.append(curvature)
        r_offset_queue.append(offset)     
    return None

def append_overall_offset(left_offset, right_offset):
    global overall_offset
    overall_offset = 0
    overall_offset = left_offset + right_offset
    overall_offset = overall_offset / 2.
    overall_offset = center_position - overall_offset
    return None

def calc_curvature(fit, img_):
    img_size= img_.shape
    y_eval = img_size[0]
    if fit is not None:
        a = fit[0] * xm_per_pix / ym_per_pix**2
        b = fit[1] * xm_per_pix / ym_per_pix
        c = fit[2] * xm_per_pix
        y = y_eval * ym_per_pix
    else:
        return None, None
    
    rad_curvature = pow(1 + (2*a*y + b)**2, 1.5) / math.fabs (2*a)
    offset = calc_offset(fit,y_eval)
    
    return offset, rad_curvature

def calc_offset(fit, y_eval):
    
    a = fit[0] * xm_per_pix / ym_per_pix**2
    b = fit[1] * xm_per_pix / ym_per_pix
    c = fit[2] * xm_per_pix
    y = y_eval * ym_per_pix
    return (a*y*y + b*y + c)

def check_and_fit(x, y, flag='L', threshold=1000):
    """
    Verify if number of pixels are satisfactory for a confident fit and then fit 
    :param x: 
    :param y: 
    :param flag: 
    :param threshold: 
    :return:
    """
    
    confidence_index = len(x)
    if IsLaneFound is False:
        threshold =500
    if confidence_index < threshold:
        fit = None
        foundFlag = False
    else:
        fit, cov = curve_fit(lambda x, a, b, c:a*x*x+b*x + c , x, y)
        foundFlag = True
    return fit, foundFlag, confidence_index


def mask_and_fit(mask, binary_warped, flag):
    """
    Mask the Images and then return the equation of the lane lines
    :return: 
    :param mask: 
    :param binary_warped: 
    :param flag: 
    :return: 
    """
    img = cv2.bitwise_and(binary_warped, binary_warped, mask=mask)
    x, y = extract_pixels(img)
    fit, foundFlag, confidence_index = check_and_fit(x, y, flag)
    return fit, foundFlag, confidence_index

def curvature_sanity(left_curvature, left_offset, right_curvature, right_offset):
    """
    Use The current values of Curvature and Offset from Left and Right Lanes 
    to decide if Lanes are sane 
    :param left_curvature: 
    :param left_offset: 
    :param right_curvature: 
    :param right_offset: 
    :return: 
    """
    if return_queue_len(flag='L') >= 1 and return_queue_len(flag='R') >= 1:
        offset = center_position - (left_offset + right_offset) / 2.
        offset_measure = np.abs(overall_offset - offset)
        return True if offset_measure < 0.2 else False
    else:
        return True



def update_lanewidth(left_fit, right_fit, img_):
    """
    Use the left and right fit 
    :return: 
    :param left_fit: 
    :param right_fit: 
    :param img_: 
    :return: 
    """
    img_size = img_.shape
    y_eval = np.linspace(0, img_size[0], 20)
    left_x = get_intercepts(left_fit, y_eval)
    right_x = get_intercepts(right_fit, y_eval)
    return np.clip(right_x - left_x, 400, 800)


def lanewidth_sanity(left_fit, right_fit, img_):
    """
    
    :return: 
    :param left_fit: 
    :param right_fit: 
    :param img_: 
    :return: 
    """
    global lane_width
    img_size = img_.shape
    ploty = np.linspace(0, img_size[0], 20)
    left_distances = np.vstack(calc_offset(left_fit, ploty)).T
    right_distances = np.vstack(calc_offset(right_fit, ploty)).T
    distances = right_distances - left_distances
    lanewidth = lane_width * xm_per_pix
    min_lanewidth = np.mean(lanewidth) - 2.5 * np.std(lanewidth)
    max_lanewidth = np.mean(lanewidth) + 2.5 * np.std(lanewidth)
    passes = np.sum((min_lanewidth <= distances) & (distances <= max_lanewidth)) / len(distances[0])
    return True if passes >= 0.95 else False


def lanewidth_rationalize(left_fit, confidence_index_l, right_fit, confidence_index_r, img_):
    """

    :param left_fit: 
    :param confidence_index_l: 
    :param right_fit: 
    :param confidence_index_r: 
    :param img_: 
    :return: 
    """
    img_size = img_.shape
    y = np.linspace(0, img_size[0], 20)
    if confidence_index_l > 2. * confidence_index_r or (left_fit is not None and right_fit is None):
        x = get_intercepts(left_fit, y) + lane_width
        right_fit, cov = curve_fit(lambda x, a, b, c: a * x * x + b * x + c, y, x)
    elif confidence_index_r > 2. * confidence_index_l or (left_fit is None and right_fit is not None):
        x = get_intercepts(right_fit, y) - lane_width
        left_fit, cov = curve_fit(lambda x, a, b, c: a * x * x + b * x + c, y, x)

    return left_fit, right_fit




# Master Function that processes video image by image
def process_video(img):
    """

    :param img: 
    :return: Processed Image that is written with appropriate polygon
    """
    global isPerspectiveCompute
    global m
    global minv
    global IsLaneFound
    global fCount
    global last_mask
    global patience
    global lane_width
    global disp_left 
    global disp_right
    global lane_width
    global xm_per_pix
    global offset_from_lane_center_in_cm

    # Initialize Variables
    l_found_flag = False
    r_found_flag = False
    confidence_index_l = 0
    confidence_index_r = 0
    img_size = img.shape
    IsLaneWidthSane = False
    IsCuvatureSane = False

    # Perform Camera Calibration and get Distortion Coefficients #
    undistorted_img = undistort_image(mtx, dist, img)

    # Calculate Bird' Eye Transform #
    if not isPerspectiveCompute:
        src_, dst_ = calc_warp_points()
        bin_ex = draw_polylines(undistorted_img, src_, 5)
        m, minv = calc_transform(src_, dst_)
        isPerspectiveCompute = True

    # Get Bird's Eye View #
    warped = perspective_transform(m, undistorted_img)
    binary_warped = filter_img(warped)
    
    # Lane Search
    # Polynomial Search if Lane is Found
    if IsLaneFound:
        #Left Mask and Fit
        mask_l_poly = limited_search(binary_warped, int(35), flag='L')
        left_fit, l_found_flag, confidence_index_l = mask_and_fit(mask_l_poly, binary_warped, 'L')
        
        # Right Mask and Fit 
        mask_r_poly = limited_search(binary_warped, int(35), flag='R')
        right_fit, r_found_flag, confidence_index_r = mask_and_fit(mask_r_poly, binary_warped, 'R')
        
    # Try Blind Search if Lane is Not Found
    else:
        mask_l_poly, mask_r_poly = blind_search(binary_warped, int(35))
        #Mask and Fit Left and Right Lanes
        left_fit, l_found_flag, confidence_index_l = mask_and_fit(mask_l_poly, binary_warped, 'L')
        right_fit, r_found_flag, confidence_index_r = mask_and_fit(mask_r_poly, binary_warped, 'R')

    # Check if Lane is found after searching , verify if the detected lanes are sane

    if left_fit is not None or right_fit is not None:
        # Check sanity in combination 
        if left_fit is not None and right_fit is not None:
            IsLaneWidthSane = lanewidth_sanity(left_fit, right_fit, binary_warped)
        if not IsLaneWidthSane:
            left_fit, right_fit = lanewidth_rationalize(left_fit, confidence_index_l, 
                                                        right_fit, confidence_index_r, binary_warped)
 
        # Calculate Offset and Curvature
        left_offset, left_curvature = calc_curvature(left_fit, binary_warped)
        right_offset,right_curvature = calc_curvature(right_fit, binary_warped)
        IsCurvatureSane = True #curvature_sanity(left_curvature, left_offset, right_curvature, right_offset)
        
        if IsCurvatureSane is True:
            IsLaneFound = True
            patience = 0 
            # Append Left & Right Lane Coefficients, Curvature, offset
            append_linecoeffs(left_fit, flag='L')
            append_curvature(left_offset, left_curvature, flag='L')
            append_linecoeffs(right_fit, flag='R')
            append_curvature(right_offset, right_curvature, flag='R')
            append_overall_offset(left_offset, right_offset)
            if IsLaneWidthSane is True :
                xm_per_pix = 3.7/ np.median(lane_width)
                lane_width = update_lanewidth(left_fit, right_fit, binary_warped)
        else:
            IsLaneFound = False
            patience = patience + 1
            if ((return_queue_len(flag='L') >1 and return_queue_len(flag='R') > 1)): 
                pop_queues_left()
                
                # Left & Right Fit
                left_fit  = get_predicted_fit(flag ='L')
                right_fit  = get_predicted_fit(flag ='R')

                #Left and Right Curvature Offset
                left_offset, left_curvature = calc_curvature(left_fit, binary_warped)
                right_offset, right_curvature =calc_curvature(right_fit, binary_warped)

                #Append Coefficients , Curvature
                append_linecoeffs(left_fit , flag='L')
                append_linecoeffs(right_fit, flag='R')
                append_curvature(left_offset, left_curvature, flag='L')
                append_curvature(right_offset, right_curvature, flag='R')

                #Overall 
                append_overall_offset(left_offset, right_offset)

    # If queue length is greater than 1
    if ((return_queue_len(flag='L') >= 1 and return_queue_len(flag='R') >= 1)):         
    
        # Get the mean offset
        left_fit = get_mean_fit(flag='L')
        right_fit = get_mean_fit(flag='R')

        # Left Mean offset and Right Mean offset 
        left_mean_offset, left_mean_curvature = get_mean_curvature(flag='L')
        right_mean_offset, right_mean_curvature = get_mean_curvature(flag='R')

        # Recompute masks for masking next frame
        mask_l_poly = smoothen_masks(get_last_fit(flag='L'), binary_warped, 50)
        mask_r_poly = smoothen_masks(get_last_fit(flag='R'), binary_warped, 50)
        last_mask = cv2.bitwise_or(mask_l_poly, mask_r_poly)

        if not IsLaneFound:
            last_mask = np.ones_like(binary_warped)
            
        # Refactor, draw a polygon and unwarp the image
        ploty = np.linspace(0, img_size[1] - 1, img_size[1])
        leftx = get_intercepts(left_fit, ploty)
        rightx = get_intercepts(right_fit, ploty)
        warped_out = draw_polygon(leftx, rightx, ploty, ploty, warped)

        warped_out, offset_cm = get_lane_center_offset(leftx, rightx, ploty, ploty, warped_out) # Add error offset to output image
        offset_from_lane_center_in_cm = offset_cm # Store offset for regulator purposes

        unwarped_out = inv_perspective_transform(minv, warped_out)
        output = cv2.addWeighted(img, 0.5, unwarped_out, 0.5, 0)

        
        if fCount%5==0:
            disp_left = ceil(left_mean_curvature)
            disp_right = ceil(right_mean_curvature)
            
        font = cv2.FONT_HERSHEY_SIMPLEX
        str1 = str("LEFT CURVATURE : ") + str(disp_left)
        str2 = str("RIGHT CURVATURE : ") + str(disp_right)
        cv2.putText(output, str1, (20, 460), font, 0.6, (100, 200, 100), 1, cv2.LINE_AA)
        cv2.putText(output, str2, (340, 460), font, 0.6, (100, 200, 100), 1, cv2.LINE_AA)


        
    # If none of it is available 
    else:
        warped_out = img
        unwarped_out = img
        output = img
        font = cv2.FONT_HERSHEY_SIMPLEX
        str1 = str('No lane detected!')
        cv2.putText(output, str1, (190, 303), font, .9, (255, 255, 255), 1, cv2.LINE_AA)
        
    fCount = fCount + 1    


    #cv2.imshow('frame3',offset_img)

    return output

'''
* END CITE *****************************************************************
'''

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

    return img



def main():
    
    global offset_from_lane_center_in_cm

    cap = cv2.VideoCapture(1)
    cap.set(3,640)
    cap.set(4,480)
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # turn of auto exposure

    rospy.init_node('car_vision', anonymous=True)
    image_pub = rospy.Publisher("car_vision/image_raw", Image, queue_size=10)
    offset_pub = rospy.Publisher("car_vision/lane_center_offset_cm", Float32, queue_size=10)

    rate = rospy.Rate(10) # 30hz

    offset_msg = Float32()

    while not rospy.is_shutdown():

        ret, frame = cap.read()

        # Add lanes to img
        res = aruco_marker_detector(process_video(frame))
        res = process_video(frame)

        # Get offset from center lane
        offset_msg.data = offset_from_lane_center_in_cm

        # Convert image to ROS Image format
        #img_msg = CvBridge().cv2_to_imgmsg(res, encoding="passthrough")
        
        # Publish image
        #image_pub.publish(img_msg)

        # Publish center offset
        offset_pub.publish(offset_msg)

        #cv2.imshow('cv_img', frame)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# For debugging
'''
if __name__ == "__main__":

    cap = cv2.VideoCapture(1)
    cap.set(3,640)
    cap.set(4,480)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # turn of auto exposure

    while(True):
        # Capture image from webcam
        ret, frame = cap.read()

        # Display the image
        #cv2.imshow('frame', aruco_marker_detector(frame))
	cv2.imshow('frame', process_video(frame))
	#process_video(frame)
	#cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
'''
