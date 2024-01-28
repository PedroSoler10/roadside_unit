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

# Extract distortion coefficients
dist_coeffs_data = camera_parameters['distortion_coefficients']['data']
dist_coeffs = np.array(dist_coeffs_data).reshape(-1, 1)

# Known object dimensions (in meters or any consistent unit)
object_width = 0.5
object_height = 1.8

# Detected object dimensions in the image (in pixels)
yolo_pretrained = os.path.join(cwd, 'src/perception_pkg/yolo_pretrained/yolov8n.pt')
model = YOLO(yolo_pretrained)
image_path = os.path.join(cwd, 'src/perception_pkg/images/person_standing.jpg')
image = cv2.imread(image_path)
results = model(image, classes=0, show=True)
# print(results[0])
# print(results[0].boxes)
# print(results[0].boxes[0].xywhn[0][2])
# print(results[0].boxes[0].xywhn[0][3])
image_width = results[0].boxes[0].xywh[0][2].item()
image_height = results[0].boxes[0].xywh[0][3].item()
x1 = results[0].boxes[0].xyxy[0][0].item()
y1 = results[0].boxes[0].xyxy[0][1].item()
x3 = results[0].boxes[0].xyxy[0][2].item()
y3 = results[0].boxes[0].xyxy[0][3].item()
x2 = x3
y2 = y1
x4 = x1
y4 = y3
print(image_width, image_height)
print(x1, y1)
print(x2, y2)
print(x3, y3)
print(x4, y4)



# Assuming the width of the object is used for distance estimation
# Using the formula: (object_width * focal_length_x) / image_width
distance = (object_width * focal_length_x) / image_width
print(distance)

# If you have specific points and want to use solvePnP
# Define your object points in the real world and image points
object_points = np.array([ [0,0,0], [object_width,0,0], [object_width,object_height,0], [0,object_height,0] ], dtype="double")
image_points = np.array([ [x1,y1], [x2,y2], [x3,y3], [x4,y4] ], dtype="double")  # Coordinates in the image

# Solve PnP
(success, rotation_vector, translation_vector) = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
print(rotation_vector)
print(translation_vector)

# rotation_vector and translation_vector give the pose of the object


# Height of the camera from the floor
camera_height = 1.2

# 2D image point
image_point = np.array([x_pixel, y_pixel], dtype="double")

# Convert image point to normalized device coordinates
normalized_point = np.linalg.inv(camera_matrix) @ np.array([image_point[0], image_point[1], 1])

# Adjust for 90-degree rotation around Y-axis
rotation_matrix = np.array([[0, 0, 1],
                            [0, 1, 0],
                            [-1, 0, 0]])

adjusted_point = rotation_matrix @ normalized_point

# Calculate world coordinates
world_x = (adjusted_point[0] * camera_height) / adjusted_point[2]
world_y = 0  # Object is on the floor
world_z = (adjusted_point[2] * camera_height) / adjusted_point[2]

world_point = np.array([world_x, world_y, world_z])

print("3D world point:", world_point)