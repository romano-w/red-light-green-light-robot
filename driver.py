#!/usr/bin/env python

# COSC 281 FP
# Authors: Will, Gus, Joe, Isaac
# Date: Fall 21, 11/15/2021
# Sources:

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Driver():
    def __init__(self, frequency, default_cmd_vel_topic, default_scan_topic, linear_velocity, angular_velocity):
        self.frequency = frequency
        self._cmd_pub = rospy.Publisher(default_cmd_vel_topic, Twist, queue_size=1)
        self._odom_sub = rospy.Subscriber("odom", Odometry, self._odom_callback)
        self._laser_sub = rospy.Subscriber(default_scan_topic, LaserScan, self._laser_callback, queue_size=1)

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        self.odom = np.zeros(3)

    def _laser_callback(self, msg):
        """Processes laser message."""
        pass

    def _odom_callback(self, msg):
        """Callback to process odom."""
        self.odom[0] = msg.pose.pose.position.x
        self.odom[1] = msg.pose.pose.position.y

        orient_q = msg.pose.pose.orientation
        eulers = [orient_q.x, orient_q.y, orient_q.z, orient_q.w]
        self.odom[2] = euler_from_quaternion(eulers)[2]

    def stop(self):
        """Stops the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def translate(self, d):
        """Moves the robot forward by the value d."""
        rate = rospy.Rate(self.frequency)
        start_time = rospy.get_rostime()
        duration = rospy.Duration(d / self.linear_velocity)
        while not rospy.is_shutdown() and rospy.get_rostime() - start_time < duration:
            twist_msg = Twist()
            twist_msg.linear.x = np.sign(d) * self.linear_velocity
            self._cmd_pub.publish(twist_msg)

            rate.sleep()

    def rotate_rel(self, a):
        """Rotates the robot a degrees from the current angle."""
        a = a * (math.pi / 180)
        rate = rospy.Rate(self.frequency)
        start_time = rospy.get_rostime()
        duration = rospy.Duration(abs(a) / self.linear_velocity)
        while not rospy.is_shutdown() and rospy.get_rostime() - start_time < duration:
            twist_msg = Twist()
            twist_msg.angular.z = np.sign(a) * self.angular_velocity
            self._cmd_pub.publish(twist_msg)

            rate.sleep()

    # def rotate_abs(self, target):
    #     """Rotates the robate to the target angle wrt odom."""
    #     rate = rospy.Rate(self.frequency)
    #     sign = 1 if self.odom[2] - target < 180 else -1
    #     while not rospy.is_shutdown() and abs(target - self.odom[2]) > 0.05:
    #         twist_msg = Twist()
    #         twist_msg.angular.z = sign * self.angular_velocity
    #         self._cmd_pub.publish(twist_msg)
    #
    #         rate.sleep()

    def spin(self):
        pass

def main():
    # "Global" variables
    frequency = 10
    default_cmd_vel_topic = "cmd_vel"
    default_scan_topic = "base_scan"
    linear_velocity = 0.2 # m/s
    angular_velocity = 10 * math.pi/180 # rad/s

    rospy.init_node("driver")
    rospy.sleep(2)

    driver = Driver(frequency, default_cmd_vel_topic, default_scan_topic, linear_velocity, angular_velocity)

    rospy.sleep(2)

    # driver.translate(0.2)

    # robot.translate(1)
    # robot.rotate_rel(-30)
    # robot.rotate_abs(0)
    # for i in range(4):
    #     robot.translate(1)
    #     robot.rotate_rel(90)

    # try:
    #     driver.spin()
    # except rospy.ROSInterruptException:
    #     rospy.logerr("ROS node interruped")

if __name__ == "__main__":
    main()
