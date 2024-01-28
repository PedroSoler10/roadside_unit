#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque
import imutils

# Initialize the ROS node
rospy.init_node('tennis_ball_tracker')

# Initialize CV Bridge
bridge = CvBridge()

# Define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 100, 100)
greenUpper = (64, 255, 255)
buffer = 64
pts = deque(maxlen=buffer)

# Publisher for ball detection
ball_detection_pub = rospy.Publisher('ball_detection', Bool, queue_size=10)

# Publisher for ball position
ball_position_pub = rospy.Publisher('ball_position', Point, queue_size=10)

def image_callback(msg):
    try:
        # Convert the ROS Image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Resize the frame, blur it, and convert it to the HSV color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    ball_detection = Bool(False)

    # Only proceed if at least one contour was found
    if len(cnts) > 0:
        # Find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Only proceed if the radius meets a minimum size
        if radius > 10:
            # Update
            # Update the points queue
            pts.appendleft(center)
            ball_detection = True
            
    # Publish the center of the ball
    if ball_detection is True:
        ball_pos = Point()
        ball_pos.x = center[0]
        ball_pos.y = center[1]
        ball_pos.z = 0  # Assuming 2D tracking
        ball_position_pub.publish(ball_pos)
        rospy.loginfo("The ball x-position is: %d", center[0])
        rospy.loginfo("The ball y-position is: %d", center[1])
    else:
        rospy.loginfo("The ball is not detected")
    
    ball_detection_pub.publish(ball_detection)
    
    # Convert the modified frame back to a ROS Image message
    try:
        image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

# Subscriber to the camera image topic
image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("Shutting down Tennis Ball Tracker Node")
