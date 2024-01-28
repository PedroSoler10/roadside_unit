import cv2
import imutils
from ultralytics import YOLO
import numpy as np
import os
import yaml

# Read the YAML file
yaml_file = os.path.join(os.getcwd(), 'launch/ost_2.yaml')
with open(yaml_file, 'r') as file:
    camera_parameters = yaml.safe_load(file)

# Extract camera matrix
camera_matrix_data = camera_parameters['camera_matrix']['data']
camera_matrix = np.array(camera_matrix_data).reshape(3, 3)
focal_length_x = camera_matrix[0][0]
focal_length_y = camera_matrix[1][1]

# Extract distortion coefficients
dist_coeffs_data = camera_parameters['distortion_coefficients']['data']
dist_coeffs = np.array(dist_coeffs_data).reshape(-1, 1)

# Known object dimensions (in meters or any consistent unit)
real_width = 0.50
real_height = 1.80

# Detected object dimensions in the image (in pixels)
yolo_pretrained = os.path.join(os.getcwd(), 'src/perception_pkg/yolo_pretrained/yolov8n.pt')
model = YOLO(yolo_pretrained)
image_path = os.path.join(os.getcwd(), 'src/perception_pkg/images/person_standing.jpg')
image = cv2.imread(image_path)
image= imutils.rotate_bound(image, 90)  # Rotate 90 degrees
results = model(image, classes=0, show=True)
print("Results:\n---------------\n", results)
print("Boxes:\n-----------------\n", results[0].boxes)
print(len(results[0].boxes))

# Detected object dimensions in the image (in pixels)
image_width_pixels = results[0].boxes[0].xywh[0][2].item()
image_height_pixels = results[0].boxes[0].xywh[0][3].item()

# Calculate the distance using the width (or height)
distance_width = (focal_length_x * real_width) / image_width_pixels
distance_height = (focal_length_y * real_height) / image_height_pixels

# Average distance between width and height distances
distance = (distance_width + distance_height) / 2  # or distance_height
print("Distance from the camera", distance)

# Compute world coordinates (assuming object is at Z=0)
# world_x = undistorted_point[0]
# world_y = undistorted_point[1]
# world_z = 0  # Same height as the camera

# world_point = np.array([world_x, world_y, world_z])

# print("3D world point:", world_point)

while True:
    results = model(image, classes=0, verbose=False, show=True)
