#!/usr/bin/env python

# COSC 281 FP
# Authors: Will, Gus, Joe, Isaac
# Date: Fall 21, 11/15/2021
# Sources:



import numpy as np
import sys
import rospy
import cv2
from collections import deque
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum
import os
import time

WINDOW_SIZE = 30
HIST_THRESHOLD = 0.4
SCALE_CONST = 50



class fsm(Enum):
	LOST = 0
	GREEN_DETECTED = 1
	RED_DETECTED = 2


class Vision():
	def __init__(self):

		# Publishers
		self.image_pub = rospy.Publisher("/camera/rgb/image_debug", Image, queue_size=1)
		self.vision_pub = rospy.Publisher("/vision_info", String, queue_size=1)

		#Subscribers
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self._img_callback, queue_size=1)
		self.depth_image_sub = rospy.Subscriber("/camera/depth/image", Image, self._depth_img_callback, queue_size=1)

		#FSM
		self.state = fsm.LOST   
		
		# Image Processing
		self.bridge = CvBridge()
		self._debug_image_seq_num = 0
		self.depth_image = None
		self.green_history = deque([],maxlen=WINDOW_SIZE)
		self.red_history = deque([],maxlen=WINDOW_SIZE)


		print(cv2.__version__)

		#Misc
		self.prev_time = time.time()
		self.f_count = 0



	#A function that scales an image from a scale factor
	# img: opencv image
	# scale_factor: int: percentage to scale. eg. 50 scale to 50% of original
	# output: opencv image
	def scale_img(self,img,scale_factor):
			width = int(img.shape[1] * scale_factor / 100)
			height = int(img.shape[0] * scale_factor / 100)
			dim = (width, height)
			
			# resize image
			return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

	# Function that detects if a set of contours is a rectangle or not
	# c: opencv countour
	# Output: bool
	def is_rectangle(self,c, rectangle_confidence):
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, rectangle_confidence * peri, True)
		# Retangle Detection
		if len(approx) == 4:
			(x, y, w, h) = cv2.boundingRect(approx)
			area = w * float(h)
			return True
		else:
			return False
	

	#A function that detects if an image has a colored rectangle in it. Supports green and red rectangles
	# img: opencv image
	# detect_color: string: "red" or "green"
	def detect_color_rectangle(self,img, detect_color, rectangle_confidence=0.04):

		#Blur and convert to HSV
		blurred = cv2.GaussianBlur(img, (11,11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		if detect_color.lower() == "red":
			mask = cv2.inRange(hsv, (0, 115, 124), (12, 255, 255)) #SUDI
		elif detect_color.lower() == "green":
			mask = cv2.inRange(hsv, (49, 38, 0), (102, 255, 117))

		#Apply Mask
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		_,contours,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		if len(contours) > 0:
			c = max(contours, key=cv2.contourArea)
			
			if not self.is_rectangle(c, rectangle_confidence):
				return None

			M = cv2.moments(c)
			return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		return None


	def _depth_img_callback(self, data):
		self.depth_image = self.scale_img(self.bridge.imgmsg_to_cv2(data, "passthrough"), SCALE_CONST)

	def _img_callback(self, data):
		self.calc_fps()

		# Getting and converting image
		try:
			cv_image_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image_scaled = self.scale_img(cv_image_original, SCALE_CONST)
			
			#For Calibration
			# cv2.imwrite("./red_test.png", cv_image_original)
			# print("Written")

		except CvBridgeError as e:
			print(e)
		
		# Looking for colored rectangles
		if self.detect_color_rectangle(cv_image_scaled, "red", 0.035) is not None:

			# change state
			self.state = fsm.RED_DETECTED
			self.publish_vision_info(None, None)
			print("red")
		else:

			green = self.detect_color_rectangle(cv_image_scaled, "green",0.05)

			# Follow Mode
			if green:

				self.state = fsm.GREEN_DETECTED
				distance = self.get_object_distance(self.depth_image, green)
				pct_from_center = self.get_object_percent_center(cv_image_scaled.shape[1], green)
				
				print("person_x_y: ", green[0], green[1], "distance: ", distance, "pct from center: ", pct_from_center)

				self.publish_vision_info(pct_from_center, distance)

			# Lost Mode
			else:
				self.publish_vision_info(None, None)
				self.state = fsm.LOST

		# Publish Debug Image
		self.publish_debug_image(cv_image_scaled)



	# Function to retrieve the average distance of pixels from a bounding box
	# depth_img = opencv image
	# object_xy = (x,y)
	# Output: float in meters
	def get_object_distance(self, depth_img, object_xy, sample_area=5, simple=True):
		#Checking if bounding box is inside depth img
		# if not self.is_inside(depth_img, object_xy):
		# 	return None # TODO: Maybe throw error
		
		if simple:
			return depth_img[object_xy[1], object_xy[0]]
		
		else:
			count = 0
			for x in range(object_xy[0]-sample_area,object_xy[0]+sample_area):
				for y in range(object_xy[1]-sample_area,object_xy[1]+sample_area):
					count += depth_img[y,x]
			
			pixel_count = pow(sample_area*2,2)
			return count / pixel_count
   

	# Function to calculate percentage from the center (of image) an object is
	# img_width: float
	# object_xy = (x,y)
	# Output: float. percentage from center of image: - = left. + = right
	def get_object_percent_center(self, img_width, object_xy):
		img_center = img_width // 2
		dist_from_center = object_xy[0] - img_center
		return (float(dist_from_center) / float(img_center)) * 100

	def is_valid_history(self,history):
		return

	def publish_vision_info(self, pct_from_center, distance_to_object):
		out_msg = String()
		lost = "True" if self.state == fsm.LOST else "False"
		red = "True" if self.state == fsm.RED_DETECTED else "False"
		
		pct = "None" if pct_from_center == None else str(pct_from_center)
		dst =  "None" if distance_to_object == None else str(distance_to_object)

		out_msg.data = pct + "," + dst + "," + red + "," + lost

		self.vision_pub.publish(out_msg)


	def publish_debug_image(self, img):
		try:
			img_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
			img_out.header.frame_id = "camera_rgb_optical_frame"
			img_out.header.seq = self._debug_image_seq_num
			self.image_pub.publish(img_out)
			self._debug_image_seq_num += 1
		except CvBridgeError as e:
			print(e)


	def main_loop(self):
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			# self.publish_vision_info(None, None)
			rate.sleep()

	def calc_fps(self):
		self.f_count += 1
		if self.f_count % 50 == 0:
			print("fps= " + str(100 / (time.time()-self.prev_time) ))
			self.prev_time = time.time()


def main():
	vision = Vision()
	rospy.init_node("vision_node", anonymous=True)

	try:
		# rospy.spin()
		vision.main_loop()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interruped")

	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
