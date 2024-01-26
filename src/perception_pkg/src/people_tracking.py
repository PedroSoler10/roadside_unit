#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PersonDetector:
    def __init__(self):
        self.node_name = "person_detector"
        rospy.init_node(self.node_name)
        image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
        self.bridge = CvBridge()
        # If using a pre-trained model, load it here

    def callback(self, data):
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Process the frame for person detection
        # ...

if __name__ == '__main__':
    detector = PersonDetector()
    rospy.spin()
