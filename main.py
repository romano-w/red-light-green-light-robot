#!usr/bin/env python

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
    # "Global" variables
    frequency = 10
    default_cmd_vel_topic = "cmd_vel"
    default_scan_topic = "base_scan"
    linear_velocity = 0.2 # m/s
    angular_velocity = 10 * math.pi/180 # rad/s

    rospy.init_node("main")
    rospy.sleep(2)

    rospy.init_node("driver")
    rospy.sleep(2)

    driver = Driver(frequency, default_cmd_vel_topic, default_scan_topic, linear_velocity, angular_velocity)

    rospy.sleep(2)

    driver.translate(0.2)

    # robot.translate(1)
    # robot.rotate_rel(-30)
    # robot.rotate_abs(0)
    # for i in range(4):
    #     robot.translate(1)
    #     robot.rotate_rel(90)

    # rospy.spin()

if __name__ == "__main__":
    main()
