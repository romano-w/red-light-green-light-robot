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
LINEAR_VELOCITY = .1 # m/s
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
	def __init__(self, frequency=FREQUENCY, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE, goal_following_distance=GOAL_FOLLOWING_DISTANCE):

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
		self.fsm = fsm.LOST

		self.odom = np.zeros(3)

		# Flag used to control the behavior of the robot.
		self._close_obstacle = False # Flag variable that is true if there is a close obstacle.
		self.face_detected = False # Flag for if the robot should stop for face

		# PD gain values
		self._linear_kp = 1
		self._linear_kd = 100
		self._linear_control = 0.0 # current control message for linear velocity
		self._linear_error = 0.0 # current linear error

		self._angular_kp = 1
		self._angular_kd = 100
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
        self._wall_p = kp               # Proportional term weight
        self._wall_d = kd               # Derivative term weight
		self._wall_k = k                         # K offset constant term
        self._wall_errs = []            # Error history

		self._wall_follow_distance = 0.2 # m

        self.left_scan_angle = [80.0 / 180 * math.pi, 100.0 / 180 * math.pi]
        self.right_scan_angle = [-100.0 / 180 * math.pi, -80.0 / 180 * math.pi]
        self.front_scan_angle = [-25.0 / 180 * math.pi, 25.0 / 180 * math.pi]
        self.theta = 20                 # The theta values between min and max scan angles (both same)

        self._wall_e = 0

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
		self.target_off_center = float(data[0])
		self.distance_from_goal = float(data[1])
		if data[2] == "True":
			self.fsm = fsm.FACE
		elif data[3] == "True":
			self.fsm = fsm.LOST
		elif self.fsm != fsm.AVOID:
			self.fsm = fsm.MOVE

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

	def _w_follow_pid_angle(self):
        """Calculate the angle to turn for this step.
        return -- rotation angle
        """
        self._wall_errs.append()
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

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)

	def spin(self):
		rate = rospy.Rate(self.frequency)
		while not rospy.is_shutdown():
			if self.fsm == fsm.MOVE:
				linear_error = self.goal_following_distance - self.distance_from_goal
				self.update_linear(linear_error)
				angular_error = self.target_off_center
				self.update_angular(angular_error)
				linear_vel = self._linear_control
				angular_vel = self._angular_control
				self.move(linear_vel, angular_vel)
			elif self.fsm == fsm.AVOID:
				# Turn until the obstacle is out of the way
				if self._close_obstacle is True:
					self.move(0, -self.angular_velocity)
					self._close_obstacle = False
				# Then start traversing the obstacle
				# We will get a signal from the vision.py once the person
				# is back in sight of the camera
				elif not self._close_obstacle:		# Wall follow mode
					rot = self._w_follow_pid_angle()
	                self.move(self.linear_velocity, rot)
			elif self.fsm == fsm.FACE:
				self.move(0,0)
			elif self.fsm == fsm.LOST:
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
