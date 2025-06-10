#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from math import sin, cos, pi, atan2, sqrt


class IK:
    def __init__(self):
        rospy.init_node('inverse_kinematics')

        rospy.Subscriber("platform_position", Point, self.callback_control)

        self.pub = rospy.Publisher('joint_angle', Point, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        
        
        # Parametros geometricos del robot Delta en metros
        self.l1 = 0.750  # Longitud del brazo inferior
        self.l2 = 0.850  # Longitud del brazo superior
        self.b = 0.500   # Radio de la base
        self.p = 0.250   # Radio de la plataforma movil

        self.alpha = [0, 2*pi/3, 4*pi/3]  # 0, 120, 240

        self.tilt_x = 0
        self.tilt_y = 0
        self.height = 0

        self.th = [0, 0, 0]  # angulos de las articulaciones activas
        self.ph = [0.0, 0.0, 0.0]  # angulos de las articulaciones pasivas

        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            msg = Point()
            self.IK()
            msg.x = self.th[0] * 180/pi
            msg.y = self.th[1] * 180/pi
            msg.z = self.th[2] * 180/pi
            self.pub.publish(msg)
            self.rate.sleep()

    def IK(self):
        """
        Calcula la cinematica inversa
        In: roll, pitch, self.height
        Out: self.th[th1 th2 th3]
        """
        if None in (self.tilt_x, self.tilt_y, self.height):
            return
        
        # Implementacion de las ecuaciones 3.19 y 3.20 de la tesis
        
        # Primero calculamos la orientacion de la plataforma
        psi_x = self.tilt_x
        psi_y = self.tilt_y
        
        # Calculamos psi_z usando la ecuacion 3.13
        if (cos(psi_x) + cos(psi_y)) != 0:
            psi_z = atan2(-sin(psi_x)*sin(psi_y), cos(psi_x) + cos(psi_y))
        else:
            psi_z = 0.0

        # Matriz de rotacion (ecuacion 3.12)
        R = [
            [cos(psi_y)*cos(psi_z),
                -cos(psi_y)*sin(psi_z),
                sin(psi_y)],

            [sin(psi_x)*sin(psi_y)*cos(psi_z)+cos(psi_x)*sin(psi_z), 
                cos(psi_x)*cos(psi_z)-sin(psi_x)*sin(psi_y)*sin(psi_z), 
                -sin(psi_x)*cos(psi_y)],

            [sin(psi_x)*sin(psi_z)-cos(psi_x)*cos(psi_z)*sin(psi_y), 
                sin(psi_x)*cos(psi_z)+cos(psi_x)*sin(psi_y)*sin(psi_z), 
                cos(psi_x)*cos(psi_y)]
        ]
            
        # Posicion de la plataforma (ecuaciones 3.7, 3.11)
        O7_x = self.p * (R[0][0] - R[1][1]) / 2
        O7_y = -self.p * R[0][1]

        #O7_x = self.p * (cos(psi_x) - cos(psi_y)) / 2
        #O7_y = -self.p * sin(psi_x) * cos(psi_z)

        O7_z = self.height
        
        
        
        # Posiciones de las articulaciones esfericas (O7j)
        O7j_positions = []
        for j in range(4, 7):  # j = 4,5,6
            alpha_j = self.alpha[j-4] if j < 6 else 4*pi/3
            O7j_x = O7_x + self.p * (R[0][0]*cos(alpha_j) + R[0][1]*sin(alpha_j))
            O7j_y = O7_y + self.p * (R[1][0]*cos(alpha_j) + R[1][1]*sin(alpha_j))
            O7j_z = O7_z + self.p * (R[2][0]*cos(alpha_j) + R[2][1]*sin(alpha_j))
            O7j_positions.append([O7j_x, O7j_y, O7j_z])
        
        # Resolver para cada pierna (i = 1,2,3)
        for i in range(3):
            O7j_x, O7j_y, O7j_z = O7j_positions[i]
            
            # Coeficientes (ecuaciones 3.18)
            A = 2 * self.l1 * cos(self.alpha[i]) * (self.b * cos(self.alpha[i]) - O7j_x)
            B = 2 * self.l1 * O7j_z * (cos(self.alpha[i])**2)
            C = (O7j_x**2 - 2 * self.b * O7j_x * cos(self.alpha[i]) + 
                    (cos(self.alpha[i])**2) * (self.b**2 + self.l1**2 - self.l2**2 + O7j_z**2))
            
            # Solucion para theta_i (ecuacion 3.19)
            discriminant = A**2 + B**2 - C**2
            if discriminant < 0:
                print("Posicion fuera del espacio de trabajo alcanzable")
                return
                
            sqrt_discriminant = sqrt(discriminant)
            denominator = C - A
            
            if abs(denominator) > 1e-6:  # Evitar division por cero
                # Tomamos la solucion "hacia afuera" para evitar colisiones
                self.th[i] = int(2 * atan2(-B + sqrt_discriminant, denominator))
            else:
                self.th[i] = int(pi/2 if B > 0 else -pi/2)
                
            # Calcular phi_i (ecuacion 3.20)
            c_phi = (O7j_x - cos(self.alpha[i]) * (self.b + self.l1 * cos(self.th[i]))) / (self.l2 * cos(self.alpha[i]))
            s_phi = -(O7j_z + self.l1 * sin(self.th[i])) / self.l2
            
            self.ph[i] = atan2(s_phi, c_phi)


        # print(f"{self.th[0]*180/pi:.2f}\t{self.th[1]*180/pi:.2f}\t{self.th[2]*180/pi:.2f}")
        

    def callback_control(self, msg):
        self.tilt_x = msg.x
        self.tilt_y = msg.y
        self.height = msg.z


def main():
    try:
        invKin = IK()
    except Exception as error: print(error)
    except rospy.ROSInterruptException:
        print("Node Terminated!")
    

if __name__ == '__main__':
    main()
