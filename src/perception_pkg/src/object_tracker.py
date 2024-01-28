#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
from ultralytics import YOLO
from ultralytics.solutions import speed_estimation
import numpy as np
import os
import yaml
import signal
import sys
import matplotlib.pyplot as plt
from datetime import datetime


class ObjectTracker:
    def __init__(self):
        
        # Initialize the ROS node
        rospy.init_node('object_tracker', anonymous=True)

        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Load the YOLO model
        yolo_pretrained = os.path.join(os.getcwd(), 'src/perception_pkg/yolo_pretrained/yolov8n.pt')
        self.model = YOLO(yolo_pretrained)
        
        # Initialize speed-estimation object
        # line_pts = [(0, 360), (1280, 360)]
        # names = self.model.names
        # self.speed_obj = speed_estimation.SpeedEstimator()
        # self.speed_obj.set_args(reg_pts=line_pts,
        #                names=names,
        #                view_img=True)

        # Configure the signal handler to get SIGINT
        signal.signal(signal.SIGINT, self.signal_handler)

        # Initialize the position estimator
        self.init_pose_estimator()

        # Initialize the vectors for the plot
        self.t = []
        self.dist = []

        # Create a subscriber to the camera topic
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)

    def before_shutdown(self):
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
        filename = f"distance_plot_{formatted_time}.png"
        plot_path = os.path.join(os.getcwd(), 'src/perception_pkg/plots/', filename)
        plt.savefig(plot_path)

        # Show directly the plot
        plt.show()
        
    def signal_handler(self, sig, frame):
        self.before_shutdown()
        sys.exit(0)

    def init_pose_estimator(self):
        # Read the YAML file
        yaml_file = os.path.join(os.getcwd(), 'launch/ost_2.yaml')
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
        try:
            # Convert the image from ROS to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image= imutils.rotate_bound(cv_image, 90)  # Rotate 90 degrees
        except CvBridgeError as e:
            print(e)

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = self.model.track(source=cv_image,
                                   persist=True,
                                   #tracker="bytetrack",
                                   conf=0.3,
                                   #iou=0.5,
                                   classes=0,
                                   #verbose=True,
                                   #visualize=True,
                                   show=True
                                   )
        
        # Detections for the image
        detections = results[0].boxes

        # Check if there are any detections
        if len(detections) > 0:
            print("At least one object detected.")

            # Find the detection with the highest confidence
            max_confidence = 0
            best_detection = None
            for detection in detections:
                if detection.conf.item() > max_confidence:
                    max_confidence = detection.conf.item()
                    best_detection = detection

            if best_detection is not None:
                print("Detection with highest confidence has id:", best_detection.id.item(), ", and confidence of:", best_detection.conf.item())
                # best_detection now contains the bounding box, confidence, and class of the highest confidence detection
                
                # Detected object dimensions in the image (in pixels)
                image_width_pixels = best_detection.xywh[0][2].item()
                image_height_pixels = best_detection.xywh[0][3].item()

                # Calculate the distance using the width (or height)
                distance_width = (self.focal_length_x * self.real_width) / image_width_pixels
                distance_height = (self.focal_length_y * self.real_height) / image_height_pixels

                # Average distance between width and height distances
                distance = (distance_width + distance_height) / 2
                print("Distance from the camera", distance)
                
                # Store the time and distance values for the plot
                current_time = rospy.Time.now()
                current_time_float = current_time.to_sec()
                self.t.append(current_time_float)
                self.dist.append(distance)
        else:
            print("No objects detected.")

        # cv_image = self.speed_obj.estimate_speed(cv_image, results)

        # Visualize the results on the frame
        # annotated_frame = results[0].plot()

        # Display the annotated frame
        # cv2.imshow("YOLOv8 Tracking", cv_image)

def main():
    object_tracker = ObjectTracker()
    rospy.spin()

if __name__ == '__main__':
    main()
