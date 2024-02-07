#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String, Float64
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D 

import numpy as np
import os
import yaml
import matplotlib.pyplot as plt
from datetime import datetime

class DistanceEstimator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('distance_estimator')

        # Get parameters
        detections_topic = rospy.get_param('detections_topic', '/detectnet/detections')
        self.visualization_flag = rospy.get_param('distance_visualization_flag', True)

        # Initialize the position estimator
        self.init_pose_estimator()

        # # Initialize the vectors for the plot
        self.t = []
        self.dist = []

        # Init thresholds
        self.image_resolution = 720.0*1280.0    # 720*1080=777600
        up_t_i = 0.3
        low_t_i = 0.2
        up_t_c = 0.8
        low_t_c = 0.5
        self.upper_importance_threshold = up_t_i * up_t_c       # 0.3*0.8=0.24
        self.lower_importance_threshold = low_t_i * low_t_c     # 0.2*0.5=0.1

        # Create a Publisher for the distance topic
        self.publisher = rospy.Publisher('/distance', Float64, queue_size=10)

        # Create a Subscriber for the detections topic
        self.subscriber = rospy.Subscriber(detections_topic, Detection2DArray, self.callback)

        # Call the function plot_before_shutdown before the node dies
        rospy.on_shutdown(self.plot_before_shutdown)

    def plot_before_shutdown(self):
        print("...Function before shutdown running...")
        print("......................................")
        print("......................................")
        
        # Create the plot
        plt.plot(self.t, self.dist)
        # print("TIME:\n\n", self.t)
        # print("DISTANCE:\n\n", self.dist)
        plt.title('Object distance to the camera')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')

        # Save the plot as an image
        # Get current date and time
        current_time = datetime.now()

        # Format as a string suitable for a filename
        formatted_time = current_time.strftime("%Y%m%d_%H%M%S")

        # Create filename
        filename = "distance_plot_" + formatted_time + ".png"
        plot_path = os.path.join(os.getcwd(), 'src/perception_pkg/plots/', filename)
        plt.savefig(plot_path)

        # Show directly the plot
        # plt.show()

    def init_pose_estimator(self):
        # Read the YAML file
        yaml_file = os.path.join(os.getcwd(), 'src/perception_pkg/camera_calibration/c505e.yaml')
        with open(yaml_file, 'r') as file:
            camera_parameters = yaml.safe_load(file)

        # Extract camera matrix
        camera_matrix_data = camera_parameters['camera_matrix']['data']
        camera_matrix = np.array(camera_matrix_data).reshape(3, 3)
        self.focal_length_x = camera_matrix[0][0]
        self.focal_length_y = camera_matrix[1][1]

        # Known object dimensions (in meters or any consistent unit)
        self.real_width = 0.50
        self.real_height = 1.80

    def callback(self, data):
        
        main_human_detection = Detection2D()
        max_importance = 0.0

        for detection in data.detections:
            if detection.results[0].id == 1:
                print("Human detected")
                importance = detection.bbox.size_x * detection.bbox.size_y * detection.results[0].score / self.image_resolution
                if importance > max_importance:
                    print("Pedestrian detected")
                    max_importance = importance
                    main_human_detection = detection
        
        if max_importance > self.lower_importance_threshold:
            # Detected object dimensions in the image (in pixels)
            image_width_pixels = main_human_detection.bbox.size_x
            image_height_pixels = main_human_detection.bbox.size_y

            # Calculate the distance using the width (or height)
            distance_width = (self.focal_length_x * self.real_width) / image_width_pixels
            distance_height = (self.focal_length_y * self.real_height) / image_height_pixels

            # Average distance between width and height distances
            # distance = (distance_width + distance_height) / 2
            self.distance = distance_width
            print("Distance from the camera", self.distance)

            # Publish the distance
            self.publisher.publish(self.distance)
            rospy.loginfo("Published: %s", self.distance)

            if self.visualization_flag:
                # Store the time and distance values for the plot
                current_time = rospy.Time.now()
                current_time_float = current_time.to_sec()
                self.t.append(current_time_float)
                self.dist.append(self.distance)
        else:
            print("No objects detected.")
            
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DistanceEstimator()
        node.run()
    except rospy.ROSInterruptException:
        pass
