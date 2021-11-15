#!/usr/bin/env python

# COSC 281 FP
# Authors: Will, Gus, Joe, Isaac
# Date: Fall 21, 11/15/2021
# Sources:

import rospy
import math
import numpy as np
from enum import Enum
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # on the robot it is called 'scan'

# Frequency at which the loop operates
FREQUENCY = 5 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = .1 # m/s
ANGULAR_VELOCITY = math.pi/6 # rad/s

# Threshold distances
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, minimum clearance distance for obstacles
GOAL_FOLLOWING_DISTANCE = 2.0 # m, distance to maintain from target


class fsm(Enum):
	MOVE = 1
	LOST = 2
	AVOID = 3
	FACE = 4


class Driver():
	def __init__(self, frequency=FREQUENCY, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE, goal_following_distance=GOAL_FOLLOWING_DISTANCE):

		# TODO:
		# Node for publishing everything:
			# pct_from_center: float
			# distance_to_object: float
			# face_detected: bool -> string = "True" "False"
			# lost: bool -> string = "True" "False"
			# string format: pct_from_center,distance_to_object,face_detected,lost

		self._vision_sub = rospy.Subscriber("vision_info", String, self._vision_callback)

	# Set up subscribers and publishers
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
		self._odom_sub = rospy.Subscriber("odom", Odometry, self._odom_callback)
		self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

		# Parameters.
		self.linear_velocity = linear_velocity
		self.angular_velocity = angular_velocity
		self.frequency = frequency
		self.min_threshold_distance = min_threshold_distance
		self.goal_following_distance = goal_following_distance
		self.distance_from_goal = 0.0
		self.target_off_center = 0.0
		self.loops = 0
		self.fsm = fsm.LOST

		self.odom = np.zeros(3)

		# Flag used to control the behavior of the robot.
		self._close_obstacle = False # Flag variable that is true if there is a close obstacle.
		self.face_detected = False # Flag for if the robot should stop for face


	def stop(self):
		"""Stops the robot."""
		twist_msg = Twist()
		self._cmd_pub.publish(twist_msg)

	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)

	def demo(self):
		angular = 0.1
		linear = 0.1
		rate = rospy.Rate(self._frequency)

		#Lost Mode 5 seconds
		
		start = time.time()
		runtime = 5
		while not rospy.is_shutdown():
			if start + runtime < time.time():
				break
			self.move(0,angular)
			rate.sleep()

		#Found Person stright 1 seconds
		start = time.time()
		runtime = 1
		while not rospy.is_shutdown():
			if start + runtime < time.time():
				break
			self.move(linear,0)
			rate.sleep()

		# Lost 1 second
		start = time.time()
		runtime = 1
		while not rospy.is_shutdown():
			if start + runtime < time.time():
				break
			self.move(linear,angular)
			rate.sleep()


		# FP left 1 second
		start = time.time()
		runtime = 1
		while not rospy.is_shutdown():
			if start + runtime < time.time():
				break
			self.move(linear,angular)
			rate.sleep()


		# FP right 1 second
		start = time.time()
		runtime = 1
		while not rospy.is_shutdown():
			if start + runtime < time.time():
				break
			self.move(linear,-angular)
			rate.sleep()


		# FD 4 seconds
		start = time.time()
		runtime = 4
		while not rospy.is_shutdown():
			if start + runtime < time.time():
				break
			self.move(0,0)
			rate.sleep()


		# FP straight
		start = time.time()
		runtime = 2
		while not rospy.is_shutdown():
			if start + runtime < time.time():
				break
			self.move(linear,angular)
			rate.sleep()



def main():
	rospy.init_node("driver")
	rospy.sleep(2)
	driver = Driver()

	rospy.sleep(2)

	try:
		driver.demo()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interruped")

if __name__ == "__main__":
	main()
