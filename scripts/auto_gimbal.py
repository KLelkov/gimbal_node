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
        self.Ypoi = 0
        self.Heading = 0
        self.heading_offset = 15
        self.ser = serial.Serial()
        try:
            self.ser = serial.Serial(
                port='/dev/gimbal',
                baudrate = 115200,
                parity = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
                timeout = 0.1 )
        except:
            rospy.logerr("[auto_gimbal] COM port error! Closing...")
            rospy.signal_shutdown("[auto_gimbal] COM port error!")

        self.navPub = rospy.Subscriber('navigation_data', navigation, self.navigation_callback)
        self.tarPub = rospy.Subscriber('target_data', target, self.target_callback)

        rospy.loginfo("Automatic gimbal control is ready!")
    # End of __init__()

    def pi2pi(self, angle):
        while abs(angle) > 180:
            angle = angle - 360 * angle / abs(angle)
        return angle

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
        gimbal_heading = round((peleng - self.Heading) * 180.0 / pi) - self.heading_offset
        gimbal_heading = self.pi2pi(gimbal_heading)
        distance = sqrt((self.Ypoi - self.Y)**2 + (self.Xpoi - self.X)**2)
        optimal_distance = 7  # meters
        gimbal_pitch = -round((distance - optimal_distance) * 3)
        rospy.loginfo("Debug: distance: {:.1f}".format(distance - optimal_distance))
        rospy.loginfo("Debug: g_pit: {:.1f}".format(gimbal_pitch))
        gimbal_heading = self.bound(gimbal_heading, -110, 110)
        gimbal_heading = int(gimbal_heading / 2)
        gimbal_pitch = self.bound(gimbal_pitch, -40, 20)
        self.ser.flush()
        self.ser.write("campos {} {} \n".format(gimbal_heading, gimbal_pitch).encode())
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
        #try:
        #    code, msg, val = m.getSystemState(caller_id)
        #except:
        #    break
    rospy.loginfo("[auto_gimbal_node] Closing...")
    SAP.ser.close()
# End on main()


if __name__ == "__main__":
    main()
