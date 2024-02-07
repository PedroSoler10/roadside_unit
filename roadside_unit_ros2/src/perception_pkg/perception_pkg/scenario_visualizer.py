#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2

class ScenarioVisualizer(Node):
    def __init__(self):
        super().__init__('scenario_visualizer')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Global variable to store the position of the ball
        self.ball_position = None

        # Global variable to store the scenario
        self.scenario = None

        # Publisher for the modified image
        self.image_pub = self.create_publisher(Image, '/scenario/overlay', 10)

        # Subscribers to the camera image and scenario topics
        self.image_sub = self.create_subscription(Image, '/detectnet/overlay', self.image_callback, 10)
        self.scenario_sub = self.create_subscription(String, '/scenario', self.scenario_callback, 10)

    # Callback for scenario updates
    def scenario_callback(self, msg):
        self.scenario = "Road state is: " + msg.data

    # Callback for image messages
    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
            return

        if self.scenario is not None:
            # Add text to the image
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_position = (50, 50)
            font_scale = 1
            font_color = (255, 255, 255)
            line_type = 2
            cv2.putText(frame, self.scenario, text_position, font, font_scale, font_color, line_type)
            self.get_logger().info("The text is included")

            # Convert the modified frame back to a ROS Image message and publish it
            image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    visualizer = ScenarioVisualizer()
    rclpy.spin(visualizer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
