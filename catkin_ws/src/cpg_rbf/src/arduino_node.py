#!/usr/bin/env python3

import rospy
from math import fabs
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

goal_pos = 0;
pub = rospy.Publisher('frequency', Float64MultiArray)

def transform_callback(data):
    global goal_pos

    pressure = data.data
    pub.publish(Float64MultiArray(goal_pos))


def dxl_control():
    rospy.init_node('dxl_control', anonymous=True)
    rospy.Subscriber('/tilt_controller/state', JointState, transform_callback).
    # Initial movement.
    pub.publish(Float64MultiArray(goal_pos))
    rospy.spin()


if __name__ == '__main__':
    try:
        dxl_control()
    except rospy.ROSInterruptException:
        pass