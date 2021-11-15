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
import cv3
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum
import os

class fsm(Enum):
    LOST = 0
    PERSON_DETECTED = 1
    FACE_DETECTED = 2


class Vision():
    def __init__(self):

        #TODO:
        # Node for publishing everything:
            # pct_from_center: float
            # distance_to_object: float
            # face_detected: bool -> string = "True" "False"
            # lost: bool -> string = "True" "False"
            # string format: pct_from_center,distance_to_object,face_detected,lost

        self.image_pub = rospy.Publisher("/camera/rgb/image_debug", Image, queue_size=1)
        self.vision_pub = rospy.Publisher("/vision_info", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self._img_callback, queue_size=1)

        dir_name = "/home/husarion/husarion_ws/src/red-light-green-light-robot/vizfiles"
        print(cv2.__version__)

        self.state = fsm.LOST   # robot is initially lost
        # self.face_cascade = cv2.CascadeClassifier(dir_name + '/haarcascade_frontalface_default.xml')    # load face cascade
        
        # code for loading yolo
        labelsPath = dir_name + "/coco.names"
        weightsPath = dir_name + "/yolov3-tiny.weights"
        configPath = dir_name + "/yolov3.cfg"
        self.LABELS = open(labelsPath).read().strip().split("\n")
        # self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
        
        # self.layer_names = self.net.getLayerNames()
        # self.layer_names = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        self._debug_image_seq_num = 0

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
        
        if False and self.face_found(cv_image):
            # change state
            self.state = fsm.FACE_DETECTED
        else:
            # person = self.find_person(cv_image)
            # change state
            person = [100,50]
            if person:
                #### Do something with these coordinates
                personX = person[0]
                personY = person[1]
                self.state = fsm.PERSON_DETECTED
                cv_image = image = cv2.putText(cv_image, 'Joe uses f strings in python2', (50,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            else:
                self.state = fsm.LOST

        try:
            
            img_out = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
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
   

    # Function to calculate percentage from the center (of image) an object is
    # img_width: float
    # object_xy = (x,y)
    # Output: float. percentage from center of image: - = left. + = right
    def get_object_percent_center(self, img_width, object_xy):
        img_center = img_width // 2
        dist_from_center = object_xy[0] - img_center
        return (dist_from_center / img_center) * 100


    def publish_vision_info(self, pct_from_center, distance_to_object):
        out_msg = String()
        out = str(pct_from_center) + "," + str(distance_to_object) + "," + 



def main():
    vision = Vision()
    rospy.init_node("vision_node", anonymous=True)

    try:
        rospy.spin()
        # vision.main_loop()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interruped")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
