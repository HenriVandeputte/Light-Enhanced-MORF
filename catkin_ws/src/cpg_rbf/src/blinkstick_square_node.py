#!/usr/bin/env python3
from __future__ import print_function, division
import sys

import rospy
from std_msgs.msg import Int16, String, ColorRGBA
from blinkstick import blinkstick


class blinkstickDriver(object):

    def __init__(self):

        try:
            self.bstick = blinkstick.find_first()
            self.bstick.morph(channel=0, index=0, red=100, green=65, blue=0)
        except:
            rospy.logerr('unable to find BlinkStick Square')
            sys.exit(-1)
        rospy.loginfo('connected to BlinkStick Square')
    
    # rospy.Subscriber("set_all_led", ColorRGBA, self.set_all)
    # rospy.Subscriber("set_single_led", ColorRGBA, self.set_single)
    
    # def set_all(self,data):
	#     self.bstick.morph(channel=0, index=0, red=data.r, green=data.g, blue=data.b)

    # def set_single(self,data):
	#     self.bstick.morph(channel=0, index=int(data.a), red=data.r, green=data.g, blue=data.b)


def main():
    rospy.init_node('blinkstick_square')
    node = blinkstickDriver()
    rospy.spin()

if __name__ == '__main__':
    main()
