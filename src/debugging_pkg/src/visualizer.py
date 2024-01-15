#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Initialize the ROS node
rospy.init_node('visualizer')

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

# Callback for ball position updates
def ball_position_callback(msg):
    global ball_position
    ball_position = (int(msg.x), int(msg.y))

# Callback for image messages
def image_callback(msg):
    global ball_position

    if ball_position is None:
        return

    try:
        # Convert the ROS Image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Draw the dot on the frame
        cv2.circle(frame, ball_position, 5, (0, 0, 255), -1)  # Red dot of radius 5
        rospy.loginfo("Red dot is drawn")

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

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Publisher for the modified image
image_pub = rospy.Publisher('tennis_ball_image', Image, queue_size=10)

# Subscribers to the camera image and ball position topics
image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
ball_position_sub = rospy.Subscriber("/ball_position", Point, ball_position_callback)
scenario_sub = rospy.Subscriber("scenario", String, scenario_callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("Shutting down visualizer")
