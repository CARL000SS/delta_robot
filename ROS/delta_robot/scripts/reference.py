#!/usr/bin/env python

#
#
# rostopic pub /select_reference std_msgs/Int16 "data: 1" --once
#
#

import rospy
import time
import math
from geometry_msgs.msg import Point
from std_msgs.msg import Int16


class ReferenceNode:
    def __init__(self):
        rospy.init_node('reference')
        self.pub = rospy.Publisher('reference', Point, queue_size=10)
        self.rate = rospy.Rate(10)  # Hz

        rospy.Subscriber("select_reference", Int16, call_reference)
        self.selection = 1

        # Circular trajectory params
        self.radius = 50
        self.height = 1
        self.period = 5

        self.x = 0
        self.y = 0
        self.h = 0

        self.strDict = {
            1: "Origin",
            2: "Diagonal",
            3: "Circle",
            4: "Square",
            5: "Infinite",
            6: "Hexagon",
            7: "send motors"
        }
        
        self.funcDict = {
            1: self.origin,
            2: self.diagonalTrajectory,
            3: self.circularTrajectory,
            4: self.squareTrajectory,
            5: self.lemniscateTrajectory,
            6: self.hexagonReference,
            7: self.sendmotors
        }

    def origin(self):
        """
        Setpoint set to center of platform
        """

        self.x = 0
        self.y = 0
        self.h = 0

    def diagonalTrajectory(self):
        """
        Genera una trayectoria diagonal
        """
        t = time.time()  # tiempo actual en segundos
        omega = 2 * math.pi / self.period  # velocidad angular [rad/s]

        self.x = self.radius * math.sin(omega * t)
        self.y = self.x
        self.h = self.height

    def circularTrajectory(self):
        """
        Genera una trayectoria circular en el plano XY con altura constante en Z.
        """
        t = time.time()  # tiempo actual en segundos
        omega = 2 * math.pi / self.period  # velocidad angular [rad/s]

        self.x = self.radius * math.cos(omega * t)
        self.y = self.radius * math.sin(omega * t)
        self.h = self.height

    def squareTrajectory(self):
        """
        Genera una trayectoria cuadrada en el plano XY con altura constante en Z.
        El movimiento es continuo y segmentado por tiempo.
        """
        t = time.time()
        T = self.period  # duracion total del ciclo
        t_mod = t % T
        side = T / 4  # tiempo en cada lado del cuadrado
        d = 5  # longitud del lado

        if t_mod < side:
            self.x = -d + 2 * d * (t_mod / side)
            self.y = -d
        elif t_mod < 2 * side:
            self.x = d
            self.y = -d + 2 * d * ((t_mod - side) / side)
        elif t_mod < 3 * side:
            self.x = d - 2 * d * ((t_mod - 2 * side) / side)
            self.y = d
        else:
            self.x = -d
            self.y = d - 2 * d * ((t_mod - 3 * side) / side)

        self.h = 1

    def lemniscateTrajectory(self):
        """
        Genera una trayectoria en forma de lemniscata (infinito) en el plano XY.
        """
        t = time.time()
        omega = 2 * math.pi / self.period
        a = 5  # amplitud

        self.x = a * math.sin(omega * t)
        self.y = a * math.sin(omega * t) * math.cos(omega * t)
        self.h = 1

    def hexagonReference(self):
        """
        Coloca puntos de referencia en los vertices de un hexagono regular.
        El punto salta de vertice en vertice cada cierto tiempo.
        """
        d = 5  # radio del hexagono
        T = self.period
        t = time.time()

        # Calcula el indice del vertice actual
        index = int((t % T) / (T / 6))  # 6 vertices

        # Lista de vertices precalculados
        vertices = [
            (d, 0),
            (d/2, d * math.sqrt(3)/2),
            (-d/2, d * math.sqrt(3)/2),
            (-d, 0),
            (-d/2, -d * math.sqrt(3)/2),
            (d/2, -d * math.sqrt(3)/2)
        ]

        self.x, self.y = vertices[index]
        self.h = 1  # altura constante

    def sendmotors(self):
        t = time.time()  # tiempo actual en segundos
        omega = 2 * math.pi / self.period  # velocidad angular [rad/s]

        self.x = 130 + 120 * math.sin(omega * t)
        self.y = self.x
        self.h = self.x

def loop():
    while not rospy.is_shutdown():
        msg = Point()

        node.funcDict[node.selection]()

        msg.x = node.x
        msg.y = node.y
        msg.z = node.h

        node.pub.publish(msg)
        node.rate.sleep()

def call_reference(msg):
    if 1 <= msg.data <= 7:
        rospy.loginfo("Reference set to: " + node.strDict[msg.data])
        node.selection = msg.data
    else:
        rospy.loginfo("Select a reference from 1 to 5.\nReference set to Origin")
        node.selection = 1


def main():
    try: loop()
    except Exception as error: print(error)
    except rospy.ROSInterruptException:
        print("Node Terminated!")

if __name__ == '__main__':
    node = ReferenceNode()
    main()
