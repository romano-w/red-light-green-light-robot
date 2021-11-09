#!/usr/bin/env python

# COSC 281 FP
# Authors: Will, Gus, Joe, Isaac
# Date: Fall 21, 11/15/2021
# Sources:

from driving import Driver

from enum import Enum
import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class fsm(Enum):
    """Overrides enum to store state."""
    MOVE_FORWARD = 1
    ROTATE = 2
    STOP = 3


class Robot():
    def __init__(self):
        pass

    def spin(self):
        rate = rospy.Rate(self._frequency)
        while not rospy.is_shutdown():
            pass

def main():

    rospy.init_node("main")
    rospy.sleep(2)

    robot = Robot()

    robot.spin()

if __name__ == "__main__":
    main()
