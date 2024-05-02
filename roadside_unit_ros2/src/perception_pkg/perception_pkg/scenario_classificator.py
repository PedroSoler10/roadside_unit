#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D
from datetime import datetime

class ScenarioClassificator(Node):
    def __init__(self):
        super().__init__('scenario_classificator')

        detections_topic = self.declare_parameter('detections_topic', '/detectnet/detections').value
        self.visualization_flag = self.declare_parameter('scenario_visualization_flag', True).value

        self.publisher = self.create_publisher(String, '/scenario', 10)

        # Init thresholds
        self.image_resolution = 720.0 * 1280.0
        up_t_i = 0.3
        low_t_i = 0.2
        up_t_c = 0.8
        low_t_c = 0.5
        self.upper_importance_threshold = up_t_i * up_t_c
        self.lower_importance_threshold = low_t_i * low_t_c

        # Init metrics
        self.time, self.width, self.height, self.size, self.conf, self.imp = [], [], [], [], [], []

        self.scenario = "unknown"
        self.subscriber = self.create_subscription(Detection2DArray, detections_topic, self.callback, 10)

    def visualize_metrics(self, detection, stamp):
        self.time.append(stamp)
        self.width.append(detection.bbox.size_x)
        self.height.append(detection.bbox.size_y)
        self.size.append(detection.bbox.size_x * detection.bbox.size_y)
        self.conf.append(detection.results[0].score)
        self.imp.append(detection.bbox.size_x * detection.bbox.size_y * detection.results[0].score / self.image_resolution)
        self.get_logger().info(f"time:\t{stamp:.2f}\twidth:\t{self.width[-1]:.0f}\theight:\t{self.height[-1]:.0f}\tsize:\t{self.size[-1]:.0f}\tconf:\t{self.conf[-1]:.4f}\timp:\t{self.imp[-1]:.4f}")

    def callback(self, data):
        main_human_detection = Detection2D()
        max_importance = -1.0

        for detection in data.detections:
            if detection.results[0].id == "\x01":
                self.get_logger().info("Human detected")
                importance = detection.bbox.size_x * detection.bbox.size_y * detection.results[0].score / self.image_resolution
                if importance > max_importance:
                    self.get_logger().info("Pedestrian detected")
                    max_importance = importance
                    main_human_detection = detection

        if max_importance > self.upper_importance_threshold:
            self.scenario = "used"
        elif max_importance > self.lower_importance_threshold:
            self.scenario = "solicited"
        elif max_importance >= 0:
            self.scenario = "free"

        self.publisher.publish(String(data=self.scenario))
        self.get_logger().info('Published: %s' % self.scenario)

        if self.visualization_flag and self.scenario != "free" and self.scenario != "unknown":
            self.visualize_metrics(main_human_detection, data.header.stamp.sec)

def main(args=None):
    rclpy.init(args=args)
    node = ScenarioClassificator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
