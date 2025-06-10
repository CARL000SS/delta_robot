#!/usr/bin/env python

import rospy
import socket
from geometry_msgs.msg import Point

class ReferenceUDPNode:
    def __init__(self):
        rospy.init_node('udp')
        self.udp_ip = rospy.get_param("~udp_ip", "10.48.104.116")
        self.udp_port = rospy.get_param("~udp_port", 5005)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber("/reference", Point, self.callback)
        rospy.loginfo("Nodo UDP inicializado, enviando a {}:{}".format(self.udp_ip, self.udp_port))

    def callback(self, msg):
        try:
            data = "{:.3f},{:.3f},{:.3f}".format(msg.x, msg.y, msg.z)
            self.sock.sendto(data.encode('utf-8'), (self.udp_ip, self.udp_port))
        except socket.error as e:
            rospy.logerr("Error de red: {}. Matando nodo.".format(e))
            rospy.signal_shutdown("Fallo de red")

if __name__ == '__main__':
    try:
        node = ReferenceUDPNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node Terminated!")
