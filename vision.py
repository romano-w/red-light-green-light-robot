#!/usr/bin/env python

# COSC 281 FP
# Authors: Will, Gus, Joe, Isaac
# Date: Fall 21, 11/15/2021
# Sources:

# import roslib
# roslib.load_manifest('my_package')
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum
import os

SCALE_FACE_CONST = 30
SCALE_BODY_CONST = 30

class fsm(Enum):
	LOST = 0
	PERSON_DETECTED = 1
	FACE_DETECTED = 2


class Vision():
	def __init__(self):


		self.image_pub = rospy.Publisher("/camera/rgb/image_debug", Image, queue_size=1)
		self.vision_pub = rospy.Publisher("/vision_info", String, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self._img_callback, queue_size=1)
		self.depth_image_sub = rospy.Subscriber("/camera/depth/image", Image, self._depth_img_callback, queue_size=1)

		#dir_name = "./vizfiles"
		dir_name = "/home/husarion/husarion_ws/src/red-light-green-light-robot/vizfiles"
		print(cv2.__version__)

		self.state = fsm.LOST   # robot is initially lost
		self.face_cascade = cv2.CascadeClassifier(dir_name + '/haarcascade_frontalface_alt2.xml')    # load face cascade
		self.body_cascade = cv2.CascadeClassifier(dir_name + '/haarcascade_upperbody.xml')
		
		# code for loading yolo
		labelsPath = dir_name + "/coco.names"
		weightsPath = dir_name + "/yolov3-tiny.weights"
		configPath = dir_name + "/yolov3.cfg"
		self.LABELS = open(labelsPath).read().strip().split("\n")
		self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
		
		self.layer_names = self.net.getLayerNames()
		self.layer_names = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

		self._debug_image_seq_num = 0

		self.depth_image = None


	def scale_img(self,img,scale_factor):
			width = int(img.shape[1] * scale_factor / 100)
			height = int(img.shape[0] * scale_factor / 100)
			dim = (width, height)
			
			# resize image
			return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

	

	def face_found(self, img):
		# Detect faces
		faces = self.face_cascade.detectMultiScale(img, 1.1, 4)

		# do we have faces
		return len(faces) > 0

	# function implemented with help from https://www.pyimagesearch.com/2018/11/12/yolo-object-detection-with-opencv/
	def find_person(self, img):

		(H, W) = img.shape[:2]


		blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (416, 416), swapRB=True, crop=False)
		self.net.setInput(blob)
		layerOutputs = self.net.forward(self.layer_names)


		for output in layerOutputs:
			# loop over each of the detections
			for detection in output:
				# extract the class ID and confidence (i.e., probability) of
				# the current object detection
				scores = detection[5:]
				classID = np.argmax(scores)
				confidence = scores[classID]
				# filter out weak predictions
				if confidence > 0.5:
					# scale box by image size and get center
					box = detection[0:4] * np.array([W, H, W, H])
					(centerX, centerY, _, _) = box.astype("int")
					
					if self.LABELS[classID] == "person":
						return (centerX, centerY)

		return False

	def find_person_haar_cascade(self,img):
		bodies = self.body_cascade.detectMultiScale(img, 1.1, 4)
		if len(bodies) > 0:
			body = bodies[0]
			return body[0] + (body[2] // 2), body[1] + (body[3] // 2)
		else:
			return False


	def _depth_img_callback(self, data):
		self.depth_image = self.scale_img(self.bridge.imgmsg_to_cv2(data, "passthrough"), SCALE_BODY_CONST)

	def _img_callback(self, data):
		try:
			cv_image_original = self.bridge.imgmsg_to_cv2(data, "bgr8")

			gray = cv2.cvtColor(cv_image_original, cv2.COLOR_BGR2GRAY)
			cv_image_face = self.scale_img(gray, SCALE_FACE_CONST)
			cv_image_body = self.scale_img(gray, SCALE_BODY_CONST)

		except CvBridgeError as e:
			print(e)
		
		if self.face_found(cv_image_face):
			# change state
			self.state = fsm.FACE_DETECTED
			self.publish_vision_info(None, None)
			print("FaceFound")

		else:
			# person = self.find_person(cv_image)
			person = self.find_person_haar_cascade(cv_image_body)
			# person = None
			# change state
			if person:
				#### Do something with these coordinates
				self.state = fsm.PERSON_DETECTED
				distance = self.get_object_distance(self.depth_image, person)
				pct_from_center = self.get_object_percent_center(cv_image_body.shape[1], person)
				print("person_x_y: ", person[0], person[1], "distance: ", distance, "pct from center: ", pct_from_center)
				
				self.publish_vision_info(pct_from_center, distance)

			else:
				print("no person")
				self.publish_vision_info(None, None)
				self.state = fsm.LOST

		try:
			img_out = self.bridge.cv2_to_imgmsg(cv_image_original, "bgr8")
			img_out.header.frame_id = "camera_rgb_optical_frame"
			img_out.header.seq = self._debug_image_seq_num
			self.image_pub.publish(img_out)
			self._debug_image_seq_num += 1
		except CvBridgeError as e:
			print(e)


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


	def publish_vision_info(self, pct_from_center, distance_to_object):
		out_msg = String()
		lost = "True" if self.state == fsm.LOST else "False"
		face = "True" if self.state == fsm.FACE_DETECTED else "False"
		pct = "None" if pct_from_center == None else str(pct_from_center)
		dst =  "None" if distance_to_object == None else str(distance_to_object)
		out_msg.data = pct + "," + dst + "," + face + "," + lost

		self.vision_pub.publish(out_msg)

	def main_loop(self):
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			# self.publish_vision_info(None, None)
			rate.sleep()


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
