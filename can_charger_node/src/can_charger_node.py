#!/usr/bin/env python
# coding: utf-8

import can
import rospy
import struct
from std_msgs.msg import Int64MultiArray

def CAN_recv():

    # Set up node
    rospy.init_node('CAN_broadcaster', anonymous=True)
    
    # Set CAN interface
    can_interface = 'can0'
    bus = can.interface.Bus(can_interface, bustype='socketcan')

    # Publisher
    pub = rospy.Publisher('CAN_bus', Int64MultiArray, queue_size=10)
    pub_bc = rospy.Publisher('CAN_bus/battery_current', Int64MultiArray, queue_size=10)
    pub_cc = rospy.Publisher('CAN_bus/charge_current', Int64MultiArray, queue_size=10)
    pub_bv = rospy.Publisher('CAN_bus/battery_voltage', Int64MultiArray, queue_size=10)
    
    # Create new
    can_msg_bat_curr = Int64MultiArray()
    can_msg_bat_curr.data = [0,0]
    
    can_msg_chrg_curr = Int64MultiArray()
    can_msg_chrg_curr.data = [0,0]
    
    can_msg_batt_volt = Int64MultiArray()
    can_msg_batt_volt.data = [0,0]

    while not rospy.is_shutdown():   

        message = bus.recv() # Wait for new CAN message
        
        # Publish the CAN message
        if message is not None: 
            
            if message.arbitration_id is 16: # Battery current
                can_msg_bat_curr.data[0] = message.arbitration_id
                try: 
                   res = struct.unpack('<i', message.data)
                except:
                  pass
                else:
                   can_msg_bat_curr.data[1] = res[0]
                pub.publish(can_msg_bat_curr)
                pub_bc.publish(can_msg_bat_curr)
            
            if message.arbitration_id is 17: # Charger current
                can_msg_chrg_curr.data[0] = message.arbitration_id
                try: 
                   res = struct.unpack('<i', message.data)
                except:
                  pass
                else:
                   can_msg_chrg_curr.data[1] = res[0]
                pub_cc.publish(can_msg_chrg_curr)            
            
            if message.arbitration_id is 18: # Battery voltage
                can_msg_batt_volt.data[0] = message.arbitration_id
                try: 
                   res = struct.unpack('<i', message.data)
                except:
                  pass
                else:
                   can_msg_batt_volt.data[1] = res[0]
                pub_bv.publish(can_msg_batt_volt)            
    
    bus.shutdown()

if __name__ == '__main__':
    try:
        CAN_recv()
    except rospy.ROSInterruptException:
        pass

