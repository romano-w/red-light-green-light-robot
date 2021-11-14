#!/usr/bin/env python

# COSC 281 FP
# Authors: Will, Gus, Joe, Isaac
# Date: Fall 21, 11/15/2021
# Sources:

import rospy
import math
import numpy as np
from enum import Enum

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # on the robot it is called 'scan'

# Frequency at which the loop operates
FREQUENCY = 30 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = .2 # m/s
ANGULAR_VELOCITY = math.pi/6 # rad/s

# Threshold distances 
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, minimum clearance distance for obstacles
GOAL_FOLLOWING_DISTANCE = 1.5 # m, distance to maintain from target

class fsm(Enum):
	MOVE = 1
	LOST = 2
	AVOID = 3
	FACE = 4

class Driver():
	def __init__(self, frequency = FREQUENCY, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE, goal_following_distance=GOAL_FOLLOWING_DISTANCE):

		# TODO:
		# Node for publishing everything:
			# pct_from_center: float
			# distance_to_object: float
			# face_detected: bool -> string = "True" "False"
			# lost: bool -> string = "True" "False"
			# string format: pct_from_center,distance_to_object,face_detected,lost

		self._vision_sub = rospy.Subscriber("vision_node", String, self._vision_callback)

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
		self.fsm = fsm.MOVE

		self.odom = np.zeros(3)

		# Flag used to control the behavior of the robot.
		self._close_obstacle = False # Flag variable that is true if there is a close obstacle.
		self.face_detected = False # Flag for if the robot should stop for face

		# PD gain values
		self._kp = 1
		self._kd = 100

		self._linear_control = 0.0 # current control message for linear velocity
		self._angular_control = 0.0 # current control message for angular velocity
		self._linear_error = 0.0 # current linear error
		self._angular_error = 0.0 # current angular error
		self._prev_error = 0.0 # previous error
		self._dt = 1 / float(FREQUENCY)

		# Other PD values
		self._control = 0.0 # current control message
		self._error = 0.0 # current error
		self._prev_error = 0.0 # previous error
		self._dt = 1 / float(FREQUENCY)

	def _laser_callback(self, msg):
		"""Processes laser message."""
		
		for i in range(len(msg.ranges)/2):
			if msg.ranges[i] < distance_from_wall:
				distance_from_wall = msg.ranges[i]
		# self._distance_from_wall = distance_from_wall
		regions_ = {
		'right':  min(min(msg.ranges[0:250]), 10),
		'front':  min(min(msg.ranges[251:500]), 10),
		'left':   min(min(msg.ranges[501:713]), 10),
		}
		self._distance_from_wall = regions_['right']
		if not self._close_obstacle:
			if regions_['front'] < self.min_threshold_distance:
				self._close_obstacle = True
				self.fsm = fsm.AVOID

	def _odom_callback(self, msg):
		"""Callback to process odom."""
		self.odom[0] = msg.pose.pose.position.x
		self.odom[1] = msg.pose.pose.position.y

		orient_q = msg.pose.pose.orientation
		eulers = [orient_q.x, orient_q.y, orient_q.z, orient_q.w]
		self.odom[2] = euler_from_quaternion(eulers)[2]

	def _vision_callback(self, msg):
		data = msg.data.split(",")
		self.target_off_center = data[0]
		self.distance_from_goal = data[1]
		if data[2] == "True":
			self.fsm = fsm.FACE
		elif data[3] == "True":
			self.fsm = fsm.LOST
		elif self.fsm != fsm.AVOID:
			self.fsm = fsm.MOVE

	def update_linear(self, err):
		self._prev_error = self._linear_error
		self._linear_error = err
		d_term =  float((self._error - self._prev_error) / float(self._dt)) 
		self._linear_control = self._kp * self._error + self._kd * d_term
	
	def update_angular(self, err):
		# takes in a percentage from center (will also be positive or negative)
		self._prev_error = self._angular_error
		self._angular_error = err
		d_term =  float((self._error - self._prev_error) / float(self._dt)) 
		self._angular_control = self._kp * self._error + self._kd * d_term

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
	
	def spin(self):
		rate = rospy.Rate(self.frequency)
		while self.loops > 0:
			if self.fsm == fsm.MOVE:
				linear_error = self.goal_following_distance - self.distance_from_goal
				self.update_linear(linear_error)
				angular_error = self.target_off_center
				self.update_angular(angular_error)
				linear_vel = self._linear_control
				angular_vel = self._angular_control
				self.move(linear_vel, angular_vel)
			if self.fsm == fsm.AVOID:
				continue
			if self.fsm == fsm.FACE:
				self.move(0,0)
			if self.fsm == fsm.LOST:
				continue
			rate.sleep()

	def lost_mode(self):
		self.move(0, self.angular_velocity)

	def avoid_obstacle(self):
		pass

def main():
	rospy.init_node("driver")
	rospy.sleep(2)
	driver = Driver()

	rospy.sleep(2)

	# driver.translate(0.2)
	# try:
	#     driver.spin()
	# except rospy.ROSInterruptException:
	#     rospy.logerr("ROS node interruped")

if __name__ == "__main__":
	main()
