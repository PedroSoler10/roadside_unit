#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/input_image_topic", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/output_image_topic", Image, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        # Convert image to gray scale image
        processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        try:
            # Publish processed image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, encoding="mono8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    ic = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
