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


class fsm(Enum):
    LOST = 0
    PERSON_DETECTED = 1
    FACE_DETECTED = 2


class Vision():
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic", Image, self._img_callback, queue_size=1)

        dir_name = "vizfiles"

        self.state = fsm.LOST   # robot is initially lost
        self.face_cascade = cv2.CascadeClassifier('{dir_name}/haarcascade_frontalface_default.xml')    # load face cascade
        
        # code for loading yolo
        labelsPath = "{dir_name}/coco.names"
        weightsPath = "{dir_name}/yolov3-tiny.weights"
        configPath = "{dir_name}/yolov3.cfg"
        self.LABELS = open(labelsPath).read().strip().split("\n")
        self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
        
        self.layer_names = self.net.getLayerNames()
        self.layer_names = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

    def face_found(self, img):
        # Convert into grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect faces
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

        # do we have faces
        return len(faces) > 0

    # function implemented with help from https://www.pyimagesearch.com/2018/11/12/yolo-object-detection-with-opencv/
    def person_found(self, img):

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


    def _img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if self.face_found(cv_image):
            # change state
            self.state = fsm.FACE_DETECTED
        else:
            person = self.find_person(cv_image)
            # change state
            if person:
                #### Do something with these coordinates
                personX = person[0]
                personY = person[1]
                self.state = fsm.PERSON_DETECTED
            else:
                self.state = fsm.LOST

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


    # Function to retrieve the average distance of pixels from a bounding box
    # depth_img = opencv image
    # object_xy = (x,y)
    # Output: float in meters
    def get_object_distance(self, depth_img, object_xy, sample_area=5, simple=True):
        #Checking if bounding box is inside depth img
        if not self.is_inside(depth_img, object_xy):
            return None # TODO: Maybe throw error
        
        if simple:
            return depth_img[object_xy[1], object_xy[0]]
        
        else:
            count = 0
            for x in range(object_xy[0]-sample_area,object_xy[0]+sample_area):
                for y in range(object_xy[1]-sample_area,object_xy[1]+sample_area):
                    count += depth_img[y,x]
            
            pixel_count = pow(sample_area*2,2)
            return count / pixel_count
   

    # Function to calculate the angle from the center of the robot to the object
    # distance: float
    # object_xy = (x,y)
    # Output: -pi/4 = left, 0 = center, pi/4 = right
    def get_object_angle(self, distance, object_xy):
        return


def main():
    vision = Vision()
    rospy.init_node("vision_node", anonymous=True)

    try:
        # vision.spin()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interruped")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
