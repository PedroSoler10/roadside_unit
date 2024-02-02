#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Initialize the ROS node
rospy.init_node('scenario_visualizer')

# Initialize CV Bridge
bridge = CvBridge()

# Global variable to store the position of the ball
ball_position = None

# Global variable to store the position of the ball
scenario = None

# Callback for scenario updates
def scenario_callback(msg):
    global scenario
    scenario = "Road state is: " + msg.data

# Callback for image messages
def image_callback(msg):
    try:
        # Convert the ROS Image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    if scenario is not None:
        # Add text to the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_position = (50, 50)
        font_scale = 1
        font_color = (255, 255, 255)
        line_type = 2
        cv2.putText(frame, scenario, text_position, font, font_scale, font_color, line_type)
        rospy.loginfo("The text is included")

        # Convert the modified frame back to a ROS Image message and publish it
        image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_pub.publish(image_message)

# Publisher for the modified image
image_pub = rospy.Publisher('/scenario/overlay', Image, queue_size=10)

# Subscribers to the camera image and ball position topics
image_sub = rospy.Subscriber("/detectnet/overlay", Image, image_callback)
scenario_sub = rospy.Subscriber("/scenario", String, scenario_callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("Shutting down visualizer")
