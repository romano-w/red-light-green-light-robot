#!/usr/bin/env python

# COSC 281 FP
# Authors: Will, Gus, Joe, Isaac
# Date: Fall 21, 11/15/2021
# Sources:

# import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Vision():
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic", Image, self._img_callback, queue_size=1)

    def _img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Load the cascade
        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        # Convert into grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Detect faces
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        
        if len(faces) > 0:
            ### CHANGE STATE HERE ###
            pass

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


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
