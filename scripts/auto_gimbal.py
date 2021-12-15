#!/usr/bin/env python

import rospy
#import getch
import math
import time
import serial # used to exchange data with arduino controller
from rdk_msgs.msg import navigation
from rdk_msgs.msg import smart_target

# Expected trajectory:
# num: x, y -> camera heading
# 1: 0, 0 -> 90
# 2: 4, 0 -> 90
# 3: 4, 4 -> 45
# 4: 8, 8 -> 0
# 5: 8, 12 -> -45
# 6: 4, 16 -> -90
# 7: 0, 16 -> -90

ser = serial.Serial(
	port='/dev/ttyACM1',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 0.1 )


waypointsCounter = 0
currentTarget = smart_target()
X = 0
Y = 0
Heading = 0

def navigation_callback(data):
	global X, Y, Heading
	X = data.X
	Y = data.Y
	Heading = data.heading


def target_callback(data):
    global currentTarget, waypointsCounter, ser
    if (currentTarget.X != data.X) and (currentTarget.Y != data.Y):
        waypointsCounter = waypointsCounter + 1
		print ("Switched path to: {} {}".format(data.X, data.Y))
	    currentTarget = data
        if (currentTarget.X == 0) and (currentTarget.Y == 0):
            ser.write("campos {} {} \n".format(-45, -40).encode())
        if (currentTarget.X == 8) and (currentTarget.Y == 0):
            ser.write("campos {} {} \n".format(45, -40).encode())
        if (currentTarget.X == 4) and (currentTarget.Y == 4):
            ser.write("campos {} {} \n".format(90, -40).encode())
        if (currentTarget.X == 8) and (currentTarget.Y == 8):
            ser.write("campos {} {} \n".format(45, -40).encode())
        if (currentTarget.X == 8) and (currentTarget.Y == 12):
            ser.write("campos {} {} \n".format(0, -40).encode())
        if (currentTarget.X == 4) and (currentTarget.Y == 16):
            ser.write("campos {} {} \n".format(-45, -40).encode())
        if (currentTarget.X == 0) and (currentTarget.Y == 16):
            ser.write("campos {} {} \n".format(-90, -40).encode())


def publisher():
    rospy.init_node('auto_gimbal', anonymous=True)
    rospy.Subscriber('navigation_data', navigation, navigation_callback)
    rospy.Subscriber('target_data', smart_target, target_callback)
    rate = rospy.Rate(10)
    print("Gimbal auto control node is running...")

    # Create an infinite loop until shutdown signal is received
    ros.spin() # Use ros.spin() when your node doesn't do anything outside of its callbacks

    # rospy.Rate.sleep()

if __name__ == '__main__':
	publisher()
