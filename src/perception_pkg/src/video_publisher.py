#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

class VideoPublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('video_publisher_node', anonymous=True)

        # Create a publisher object
        self.publisher = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Initialize video capture with the video file
        video_file = os.path.join(os.getcwd(), 'src/perception_pkg/videos/person_walking.mp4')
        self.cap = cv2.VideoCapture(video_file)

        # Get the frame rate of the video
        fps = self.cap.get(cv2.CAP_PROP_FPS)

        # Set the publishing rate based on the video frame rate
        self.rate = rospy.Rate(fps)

    def publish_frames(self):
        # Continuously capture and publish frames
        while not rospy.is_shutdown() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Rotate the frame by 90 degrees
                rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

                try:
                    # Convert the OpenCV image to a ROS Image message
                    ros_image = self.bridge.cv2_to_imgmsg(rotated_frame, "bgr8")
                    # Publish the image
                    self.publisher.publish(ros_image)
                except CvBridgeError as e:
                    rospy.logerr("CvBridge Error: {0}".format(e))

            # Sleep to maintain the video frame rate
            self.rate.sleep()


def main():
    video_node = VideoPublisherNode()
    video_node.publish_frames()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
