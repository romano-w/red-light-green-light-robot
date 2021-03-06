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
DEFAULT_SCAN_TOPIC = 'scan'

# Frequency at which the loop operates
FREQUENCY = 5 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.25 # m/s
ANGULAR_VELOCITY = math.pi/12 # rad/s

# Threshold distances
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, minimum clearance distance for obstacles
GOAL_FOLLOWING_DISTANCE = 1 # m, distance to maintain from target


class fsm(Enum):
	MOVE = 1
	LOST = 2
	AVOID = 3
	FACE = 4
	TURN = 5


class Driver():
	def __init__(self, frequency=FREQUENCY, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE, goal_following_distance=GOAL_FOLLOWING_DISTANCE):

		# self._vision_sub = rospy.Subscriber("vision_info", String, self._vision_callback)

		# Set up subscribers and publishers
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
		self._odom_sub = rospy.Subscriber("odom", Odometry, self._odom_callback)
		self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
		self._vision_sub = rospy.Subscriber("vision_info", String, self._vision_callback)


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

		# PD gain values
		self._linear_kp = -1
		self._linear_kd = 0
		self._linear_control = 0.0 # current control message for linear velocity
		self._linear_error = 0.0 # current linear error

		self._angular_kp = -1
		self._angular_kd = 0
		self._angular_control = 0.0 # current control message for angular velocity
		self._angular_error = 0.0 # current angular error
		self._prev_error = 0.0 # previous error
		self._dt = 1 / float(FREQUENCY)

		# Other PD values
		self._control = 0.0 # current control message
		self._error = 0.0 # current error
		self._prev_error = 0.0 # previous error
		self._dt = 1 / float(FREQUENCY)

		# Wall follow PID params
		self._predicted_e_time = 1          # The amount of time in the future to calculate predicted error
		self._wall_p = 1               # Proportional term weight
		self._wall_d = 100               # Derivative term weight
		self._wall_k = 0                         # K offset constant term
		self._wall_errs = []            # Error history

		self._wall_follow_distance = 0.4 # m

		self.left_scan_angle = [80.0 / 180 * math.pi, 100.0 / 180 * math.pi]
		self.right_scan_angle = [-100.0 / 180 * math.pi, -80.0 / 180 * math.pi]
		self.front_scan_angle = [20, -20]
		self.theta = 20                 # The theta values between min and max scan angles (both same)

		self._wall_e = 0

	def _laser_callback(self, msg):
		"""Processes laser message."""
		# Processing right side
		right_min_index = int((self.right_scan_angle[0] - msg.angle_min) / msg.angle_increment)
		right_max_index = int((self.right_scan_angle[1] - msg.angle_min) / msg.angle_increment)
		right_med_index = int((right_max_index + right_min_index) / 2)
		right_a = msg.ranges[right_max_index+1]
		right_b = msg.ranges[right_med_index]
		right_c = msg.ranges[right_min_index]
		self._w_follow_error(right_a, right_b, right_c)
		# rospy.loginfo("wall error: " + str(self._wall_e))		# Uncomment to see error values

		# Detect front obstacle
		front_min_index = self.front_scan_angle[0]
		front_max_index = self.front_scan_angle[1]

		# Check for obstacle and rotate in correct direction to move out of the corner
		if np.min(msg.ranges[front_min_index:front_max_index+1]) <= self._wall_follow_distance:
			self.fsm = fsm.TURN
		elif (self.fsm != fsm.MOVE) and (self.fsm != fsm.FACE) and (self.fsm != fsm.LOST):
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
		if data[0] != "None":
			self.target_off_center = float(data[0])
			angular_error = self.target_off_center
			self.update_angular(angular_error)
		if data[1] != "None":
			self.distance_from_goal = float(data[1])
			linear_error = self.goal_following_distance - self.distance_from_goal
			self.update_linear(linear_error)
			self.fsm = fsm.MOVE
		if data[2] == "True":
			self.fsm = fsm.FACE
		elif data[3] == "True":
			self.fsm = fsm.LOST
		

	def _w_follow_error(self, a, b, c):
		"""Calculates the true distance from the wall.
		a -- the forward laser reading of the fov
		b -- the laser reading perpendicular to robot
		c -- the rear laser reading of the fov
		"""
		alpha = math.atan((a * math.cos(self.theta) - c) / (a * math.sin(self.theta)))
		instant_dist = b * math.cos(alpha)
		future_dist = instant_dist + (self._predicted_e_time * self.linear_velocity * math.sin(alpha))
		error = self._wall_follow_distance - future_dist
		self._wall_e = error

	def _w_follow_pid_angle(self, err):
		"""Calculate the angle to turn for this step.
		err -- error value
		return -- rotation angle
		"""
		self._wall_errs.append(err)
		u = self._wall_k + self._wall_p * err
		if len(self._wall_errs) > 2:
			u += self._wall_d * ((self._wall_errs[-1] - self._wall_errs[-2]) / self.frequency)
		return u

	def update_linear(self, err):
		self._prev_error = self._linear_error
		self._linear_error = err
		d_term =  float((self._linear_error - self._prev_error) / float(self._dt))
		self._linear_control = self._linear_kp * self._linear_error + self._linear_kd * d_term

	def update_angular(self, err):
		# takes in a percentage from center (will also be positive or negative)
		self._prev_error = self._angular_error
		self._angular_error = err
		d_term =  float((self._angular_error - self._prev_error) / float(self._dt))
		self._angular_control = self._angular_kp * self._angular_error + self._angular_kd * d_term

	def stop(self):
		"""Stops the robot."""
		twist_msg = Twist()
		self._cmd_pub.publish(twist_msg)

	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()
		if math.isnan(linear_vel):
			linear_vel = 0
		elif linear_vel > self.linear_velocity: # Avoids moving too fast
			linear_vel = self.linear_velocity
		elif linear_vel < -self.linear_velocity: # Avoids moving too fast
			linear_vel = -self.linear_velocity
		
		twist_msg.linear.x = linear_vel
		if angular_vel > self.angular_velocity: # Avoids spinning too fast
			angular_vel = self.angular_velocity
		elif angular_vel < -self.angular_velocity: # Avoids spinning too fast
			angular_vel = -self.angular_velocity
		twist_msg.angular.z = angular_vel

		# print("linear_vel", linear_vel, "angular_vel" ,angular_vel)
		self._cmd_pub.publish(twist_msg)

	def spin(self):
		rate = rospy.Rate(self.frequency)
		while not rospy.is_shutdown():
			if self.fsm == fsm.MOVE:
				rospy.loginfo("FOLLOWING")
				linear_vel = self._linear_control
				angular_vel = self._angular_control
				self.move(linear_vel, angular_vel)
			elif self.fsm == fsm.AVOID:
				rospy.loginfo("REROUTING")
				rot = self._w_follow_pid_angle(self._wall_e)
				self.move(self.linear_velocity, rot)
			elif self.fsm == fsm.TURN:
				# Turn until the obstacle is out of the way
				rospy.loginfo("AVOIDING FRONT OBSTACLE")
				self.move(0, -self.angular_velocity)
				# self._close_obstacle = False
			elif self.fsm == fsm.FACE:
				rospy.loginfo("RED LIGHT")
				self.move(0,0)
			elif self.fsm == fsm.LOST:
				rospy.loginfo("HELP I'M LOST")
				self.lost_mode()
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

	try:
		driver.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interruped")

if __name__ == "__main__":
	main()
