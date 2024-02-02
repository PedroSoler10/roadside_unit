#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import jetson.inference
import jetson.utils

class DepthNet:
    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('depthnet', anonymous=True)

        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # load mono depth network
        self.net = jetson.inference.depthNet()

        # depthNet re-uses the same memory for the depth field,
        # so you only need to do this once (not every frame)
        depth_field = self.net.GetDepthField()

        # cudaToNumpy() will map the depth field cudaImage to numpy
        # this mapping is persistent, so you only need to do it once
        self.depth_numpy = jetson.utils.cudaToNumpy(depth_field)

        print("depth field resolution is", depth_field.width, "x", depth_field.height, "format=", depth_field.format)

        # Create a subscriber to the camera topic
        topic_img_in = rospy.get_param('topic_img_in', '/usb_cam/image_raw')
        self.image_sub = rospy.Subscriber(topic_img_in, Image, self.callback)


    def callback(self, data):
        try:
            # Convert the image from ROS to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.net.Process(cv_image)
        jetson.utils.cudaDeviceSynchronize() # wait for GPU to finish processing, so we can use the results on CPU
        
        # find the min/max values with numpy
        min_depth = np.amin(self.depth_numpy)
        max_depth = np.amax(self.depth_numpy)
        print("min_depth: ",min_depth, "\nmax_depth: ", max_depth)

def main():
    depthnet = DepthNet()
    rospy.spin()

if __name__ == '__main__':
    main()
