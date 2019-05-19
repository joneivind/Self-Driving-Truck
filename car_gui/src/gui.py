#!/usr/bin/env python

from Tkinter import *
from math import cos, sin, pi
import numpy as np
import time
import PIL.Image
import PIL.ImageTk
import rospy
from std_msgs.msg import Int16, Float64, Int64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from filters import *
import os


###### GUI ######

dirpath = os.path.dirname(os.path.abspath(__file__))

# Create a new window
root = Tk()
root.geometry("820x480") # Window size
root.title("SINTEF RC TRUCK") # Window title
root.call('wm', 'iconphoto', root._w, PhotoImage(file=dirpath + '/img/logo.gif')) # Favicon
root.resizable(width=False, height=False)
root.configure(background='black')

# Placing the window in center of the screen
w = 820
h = 480
ws = root.winfo_screenwidth()
hs = root.winfo_screenheight()
x = (ws/2) - (w/2)
y = (hs/2) - (h/2)
root.geometry('%dx%d+%d+%d' % (w, h, x, y))

# Import the images
img = PIL.Image.open(dirpath + '/img/menu_bg_back.png')
img2 = PIL.Image.open(dirpath + '/img/menu_bg_front.png')
img3 = PIL.Image.open(dirpath + '/img/charging.png')
img4 = PIL.Image.open(dirpath + '/img/not_charging.png')

img = PIL.ImageTk.PhotoImage(img)
img2 = PIL.ImageTk.PhotoImage(img2)
img3 = PIL.ImageTk.PhotoImage(img3)
img4 = PIL.ImageTk.PhotoImage(img4)

# Create a new canvas
can = Canvas(root, width=900, height=600, bg='#000', bd=0, highlightthickness=0)

# background img
can.create_image(410, 250, image=img) 

# charger icon
charge_img = can.create_image(410, 250, image=img4) 

# Meters
line_left = can.create_line(193, 243, 193+cos(np.deg2rad(180))*123, 243+sin(np.deg2rad(0))*(-123), fill="#fff", width=4, smooth=True) # speedo
line_right = can.create_line(626, 243, 626+cos(np.deg2rad(180))*123, 243+sin(np.deg2rad(0))*(-123), fill="#fff", width=4, smooth=True) # ampmeter
#can.create_text(193, 343, fill="#666", font="Ubuntu 8", text="STEERING")
#line_amp = can.create_line(193, 243, 193+cos(np.deg2rad(180-50))*123, 243+sin(np.deg2rad(-50))*(-123), fill="#ccc", width=3, smooth=True) # steering

# Foreground img
can.create_image(410, 250, image=img2) 

# Text left meter
can.create_text(193, 208, fill="#666", font="Ubuntu 8", text="KM/H")
speed = can.create_text(193, 243, fill="#fff", font="Ubuntu 30 italic bold", text="0.0")
drv_mode = can.create_text(193, 278, fill="#49a4c1", font="Ubuntu 8", text="MANUAL")

# Text right meter
can.create_text(626, 208, fill="#666", font="Ubuntu 8", text="CURRENT")
gear = can.create_text(626, 243, fill="#fff", font="Ubuntu 30 italic bold", text="0")
can.create_text(626, 278, fill="#49a4c1", font="Ubuntu 8", text="AMP")

# Text charging
can.create_text(410, 190, fill="#666", font="Ubuntu 8 bold", text="CHARGER")
charge_status = can.create_text(410, 215, fill="#ccc", font="Ubuntu 18 bold", text="OFF")

# Text dead switch
can.create_text(410, 250, fill="#666", font="Ubuntu 8 bold", text="DEAD SWITCH")
dead_sw_status = can.create_text(410, 275, fill="#ccc", font="Ubuntu 18 bold", text="OFF")

# Text amp
#can.create_text(410, 325, fill="#666", font="Ubuntu 8 bold", text="CURRENT")
#amp_status = can.create_text(410, 360, fill="#ccc", font="Ubuntu 40 bold", text="0")
#can.create_text(410, 395, fill="#ccc", font="Ubuntu 8 bold", text="MILLIAMP")

can.pack()



###### ROS ######

# Set up node
rospy.init_node('gui_node', anonymous=True)

# Set max speed and charge
max_speed = 1.5 # m/s limit for speedometer
max_m_amp = 10000.0 # limit for milliamp meter

# Variables
current_speed = 0.0
current_steering_ang = 90.0
current_drv_mode = 0
current_gear = 0
current_charge_status = False
current_dead_sw_status = 0
current_amp = 0

# filter storage
window_size = 20
filter_data_1 = np.zeros((1, window_size))
filter_data_2 = np.zeros((1, window_size))
filter_data_3 = np.zeros((1, window_size))

def car_cmd_callback(data):
    global current_steering_ang
    global current_gear

    current_steering_ang = data.angular.z
    current_gear = data.linear.z

def joy_callback(data):
    global current_dead_sw_status
    current_dead_sw_status = data.buttons[5]

def vel_callback(data):
    global current_speed
    current_speed = data.data

def charger_status_callback(data):
    global current_charge_status
    charger_status = data.data

    if charger_status is 1:
        current_charge_status = True
    else:
        current_charge_status = False

def drv_mode_callback(data):
    global current_drv_mode
    current_drv_mode = data.data

def can_bus_callback(data):
    global current_amp
    current_amp = data.data[1]

def ros_main():
    
    global current_speed
    global max_speed
    global max_m_amp
    global current_drv_mode
    global current_gear
    global current_steering_ang
    global current_amp
    global filter_data_1
    global filter_data_2
    global filter_data_3
    global window_size

    # Setup the subscribers
    rospy.Subscriber("car_cmd", Twist, car_cmd_callback)
    rospy.Subscriber("joy", Joy, joy_callback)    
    rospy.Subscriber("velocity_real", Float64, vel_callback)
    rospy.Subscriber("aruco_charger_status", Int16, charger_status_callback)
    rospy.Subscriber("led_mode", Int16, drv_mode_callback)
    rospy.Subscriber("CAN_bus", Int64MultiArray, can_bus_callback)

    rate = rospy.Rate(100) # Hz
    
    while not rospy.is_shutdown():

        # Low pass filter steering angle
        steering_ang_smooth, filter_data_1 = low_pass_filter_avg(current_steering_ang, filter_data_1, window_size)
        speed_smooth, filter_data_2 = low_pass_filter_avg(current_speed, filter_data_2, window_size)
        amp_smooth, filter_data_3 = low_pass_filter_avg(current_amp, filter_data_3, window_size)
        
        # Update needles
        can.coords(line_left, 193, 243, 193 + cos(np.deg2rad(180-43+(266*speed_smooth/max_speed)))*123, 243 + sin(np.deg2rad(-43+(266*speed_smooth/max_speed)))*(-123))
        can.coords(line_right, 626, 243, 626 + cos(np.deg2rad(180-43+(266*amp_smooth/max_m_amp)+132))*123, 243 + sin(np.deg2rad(-43+(266*amp_smooth/max_m_amp)+132))*(-123))

        # Update text
        can.itemconfigure(speed, text=int(round(speed_smooth*3.6*14-(0.4*14),0)))        
        can.itemconfigure(gear, text=str(round((amp_smooth/1000.0),1)))
        #can.itemconfigure(amp_status, text=int(round(amp_smooth,0)))
        
        if current_drv_mode is 1 or current_drv_mode is 5:
                can.itemconfigure(drv_mode, text='MANUAL')
        elif current_drv_mode is 4 or current_drv_mode is 6:
                can.itemconfigure(drv_mode, text='RECORD')
        elif current_drv_mode is 3:
                can.itemconfigure(drv_mode, text='AUTO')

        if current_amp > 0:
                can.itemconfigure(charge_status, text='OFF', fill='#fff')
                can.itemconfigure(charge_img, image=img4)
        else:
                can.itemconfigure(charge_status, text='ON', fill='yellow')
                can.itemconfigure(charge_img, image=img3)
        
        if current_dead_sw_status is 0:
                can.itemconfigure(dead_sw_status, text='OFF')
        else:
                can.itemconfigure(dead_sw_status, text='ON')
        
        # Update canvas
        can.update()

        rate.sleep()

if __name__ == '__main__':
    try:        
        root.after(10, ros_main)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
