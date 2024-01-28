#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes

class PersonDetector:
    def __init__(self):
        self.node_name = "person_detector"
        rospy.init_node(self.node_name)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.net = cv2.dnn.readNet('yolov3.weights', 'yolov3.cfg')  # Adjust with your YOLO model path
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:  # Threshold to filter weak detections
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply Non-Max Suppression to filter overlapping bounding boxes
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        for i in indices:
            i = i[0]
            box = boxes[i]
            if class_ids[i] == 0:  # Assuming 'person' is class 0
                x, y, w, h = box
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        cv2.imshow("Image", cv_image)
        cv2.waitKey(3)

def main():
    detector = PersonDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
