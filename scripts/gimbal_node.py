#!/usr/bin/env python3

import rospy
#import getch
import math
import time
import serial # used to exchange data with arduino controller
from std_msgs.msg import String
from rdk_msgs.msg import navigation

# string.isdigit() doesnt work with negative numbers
# that's why I've made my own
def is_digit(n):
    try:
        int(n)
        return True
    except ValueError:
        return  False

ser = serial.Serial(
	port='/dev/gimbal',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 0.1 )


heading_ctrl = 0
pitch_ctrl = 0

static_offset = 15

pi = 3.141592653589793
X = 0
Y = 0
Heading = 0

def navigation_callback(data):
	global X, Y, Heading
	X = data.X
	Y = data.Y
	Heading = data.heading


def publisher():
    global X, Y, Heading, heading_ctrl, pitch_ctrl
    rospy.init_node('gimbal_node', anonymous=True)
    rospy.Subscriber('navigation_data', navigation, navigation_callback)
    rate = rospy.Rate(10)#try removing this line ans see what happens
    print("Gimbal node is running.")
    print("Type in commands in format [heading] [pitch] to set camera angles.")
    print("You can use shortened version [pitch] to set pitch angle only.")
    print("You can close this node by entering ctrl+C or Q")
    print("All other input will be ignored.")

    while not rospy.is_shutdown():
        print("")
        #something = raw_input()
        something = input()
        if something.upper() == "Q":
            print("Closing this node shortly...")
            break
        parts = something.split(" ")
        if len(parts) == 1:
            if is_digit(parts[0]):
                print("Set pitch to: {}".format(int(parts[0])))
                pitch_ctrl = int(parts[0])
                ser.write("campos {} {} \n".format(heading_ctrl, pitch_ctrl).encode())
        if len(parts) == 2:
            if is_digit(parts[0]):
                print("Set heading to: {}".format(int(parts[0])))
                heading_ctrl = int(parts[0]) - static_offset
            if is_digit(parts[1]):
                print("Set pitch to: {}".format(int(parts[1])))
                pitch_ctrl = int(parts[1])
            ser.write("campos {} {} \n".format(heading_ctrl, pitch_ctrl).encode())
            #print("campos {} {} \n".format(heading_ctrl, pitch_ctrl).encode())
            #print(parts)
        #print(something)
        #k=ord(getch.getch())
        # this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        #if ((k>=65)&(k<=68)|(k==115)|(k==113)|(k==97)):# to filter only the up , dowm ,left , right key /// this line can be removed or more key can be added to this
        #if (k>=0):
        #    rospy.loginfo(str(k))# to print on  terminal
        #    pub.publish(k)#to publish
    ser.close()


if __name__ == '__main__':
	publisher()
