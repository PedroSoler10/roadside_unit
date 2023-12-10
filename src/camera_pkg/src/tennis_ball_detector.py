#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class TennisBallDetector:
    def __init__(self):
        rospy.init_node('tennis_ball_detector', anonymous=True)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/tennis_ball_position', Image, queue_size=1)
        self.image_pub_x = rospy.Publisher('/tennis_ball_position/x', Float32, queue_size=1)
        self.image_pub_y = rospy.Publisher('/tennis_ball_position/y', Float32, queue_size=1)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convertir la imagen a espacio de color HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definir rango de color para la pelota de tenis (ajustar según sea necesario)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        # Crear una máscara
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Dibujar rectángulo alrededor de la pelota de tenis
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # ajustar el umbral de área según sea necesario
                x, y, w, h = cv2.boundingRect(contour)
                xc = x + w/2
                yc = y + h/2
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Mostrar la imagen con el rectángulo dibujado (opcional)
        cv2.imshow('Tennis Ball Detection', cv_image)
        cv2.waitKey(3)

        # Publicar la imagen procesada en un nuevo tópico
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
            self.image_pub_x.publish(xc)
            self.image_pub_y.publish(yc)
        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    try:
        tennis_ball_detector = TennisBallDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
