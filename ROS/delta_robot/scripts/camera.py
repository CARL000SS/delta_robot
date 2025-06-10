#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point

class DualCameraNode:
    def __init__(self):
        rospy.init_node('cameras')
        self.pub = rospy.Publisher('position_measure', Point, queue_size=10)
        self.rate = rospy.Rate(10)
        self.show = rospy.get_param('~show_image', True)

        # Camara cenital (arriba) en el indice 0
        self.cap_top = cv2.VideoCapture(0)
        # Camara lateral en el indice 1
        self.cap_side = cv2.VideoCapture(1)

        if self.show: print("Presiona 'q' para cerrar ventanas...")

        self.x = 0.0  # Coordenada x de la camara superior
        self.y = 0.0  # Coordenada y de la camara superior
        self.z = 0.0  # Altura desde la camara lateral

        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            self.process_top_camera()
            self.process_side_camera()

            msg = Point()
            msg.x = self.x
            msg.y = self.y
            msg.z = self.z

            self.pub.publish(msg)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            self.rate.sleep()

        self.cap_top.release()
        self.cap_side.release()
        cv2.destroyAllWindows()

    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 150, 150])  # AJUSTAR al color real
        upper_color = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        centro = None
        diametro = 0

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radio) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)

            if M["m00"] > 0:
                centro = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                diametro = 2 * radio

                if radio > 10:
                    return centro, diametro, mask, frame, c
        return None, 0, mask, frame, None

    def process_top_camera(self):
        if not self.cap_top.isOpened():
            rospy.logwarn("Camara cenital no disponible.")
            return

        ret, frame = self.cap_top.read()
        if not ret:
            rospy.logwarn("No se pudo leer imagen de la camara cenital.")
            return

        centro, diametro, mask, frame, contour = self.detect_ball(frame)

        if centro:
            self.x = 1.0 * centro[0]  # Ajustar escala
            self.y = 1.0 * centro[1]

            if self.show:
                cv2.circle(frame, centro, int(diametro / 2), (0, 255, 255), 2)
                cv2.circle(frame, centro, 5, (0, 0, 255), -1)
                cv2.putText(frame, "Cenital XY: {0}".format(centro), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Camara Cenital", frame)
                #cv2.imshow("Mascara Cenital", mask)

    def process_side_camera(self):
        if not self.cap_side.isOpened():
            rospy.logwarn("Camara lateral no disponible.")
            return

        ret, frame = self.cap_side.read()
        if not ret:
            rospy.logwarn("No se pudo leer imagen de la camara lateral.")
            return

        centro, diametro, mask, frame, contour = self.detect_ball(frame)

        if centro:
            self.z = 1.0 * centro[1]  # Solo la coordenada vertical desde lateral

            if self.show:
                cv2.circle(frame, centro, int(diametro / 2), (0, 255, 255), 2)
                cv2.circle(frame, centro, 5, (0, 0, 255), -1)
                cv2.putText(frame, "Altura Y: {0}".format(centro[1]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Camara Lateral", frame)
                #cv2.imshow("Mascara Lateral", mask)

def main():
    try:
        DualCameraNode()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
