#!/usr/bin/env python3

import rospy
from math import pi, cos, sin, atan2, sqrt
import serial # used to exchange data with arduino controller
from rdk_msgs.msg import navigation
from rdk_msgs.msg import target


class SubscribeAndPublish:

    def __init__(self):
        self.X = 0
        self.Y = 0
        self.Xpoi = 200
        self.Zpoi = 0
        self.Heading = 0

		self.ser = serial.Serial(
			port='/dev/gimbal',
			baudrate = 115200,
			parity = serial.PARITY_NONE,
			stopbits = serial.STOPBITS_ONE,
			bytesize = serial.EIGHTBITS,
			timeout = 0.1 )

        self.navPub = rospy.Subscriber('navigation_data', navigation, self.navigation_callback)
        self.tarPub = rospy.Subscriber('target_data', target, self.target_callback)

        rospy.loginfo("Automatic gimbal control is ready!")
    # End of __init__()


	def bound(self, value, mi, ma):
		upper = max([mi, value])
		lower = min([upper, ma])
		return lower

    def navigation_callback(self, msg):
        self.X = msg.X
        self.Y = msg.Y
        self.Heading = msg.heading

    def target_callback(self, msg):
        self.Xpoi = msg.Xpoi
        self.Ypoi = msg.Ypoi

    def generate_controls(self):
		peleng = atan2(self.Ypoi - self.Y, self.Xpoi - self.X)
		gimbal_heading = round((peleng - self.Heading) * 180.0 / pi)
		distance = sqrt((self.Ypoi - self.Y)**2 + (self.Xpoi - self.X)**2)
		optimal_distance = 5  # meters
		gimbal_pitch = round((distance - optimal_distance) * 5)
		gimbal_heading = self.bound(gimbal_heading, -90, 90)
		gimbal_pitch = self.bound(gimbal_pitch, -40, 20)
		ser.flush()
		ser.write("campos {} {} \n".format(gimbal_heading, gimbal_pitch).encode())
		rospy.loginfo("Sent [campos {} {}]".format(gimbal_heading, gimbal_pitch))
    # End of generate_controls()

# End of SAP class


def main():
    rospy.init_node('auto_gimbal_node')
    SAP = SubscribeAndPublish()

    rate = rospy.get_param("~rate", 0.2)
    r = rospy.Rate(rate)  # Hz
    while not rospy.is_shutdown():
        # Main cycle
        SAP.generate_controls()
        r.sleep()
	rospy.loginfo("[auto_gimbal_node] Closing...")
# End on main()


if __name__ == "__main__":
    main()
