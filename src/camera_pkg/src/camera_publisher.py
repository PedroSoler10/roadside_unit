#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(object):
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.rate = rospy.Rate(10)  # Publishing rate
        self.cap = cv2.VideoCapture(2)  # Change to 1 if using a secondary camera
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('camera_image', Image, queue_size=10)

    def publish_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(ros_image)

            self.rate.sleep()

        self.cap.release()

if __name__ == '__main__':
    try:
        image_publisher = ImagePublisher()
        image_publisher.publish_image()
    except rospy.ROSInterruptException:
        pass

