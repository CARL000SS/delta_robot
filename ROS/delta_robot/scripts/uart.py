#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
import serial
import struct

class UART:
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200):
        self.ser = None  # Inicializar como None para manejar errores
        self.th = [0, 0, 0]  # Valores predeterminados
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            rospy.loginfo("Puerto {0} abierto correctamente".format(port))
        except serial.SerialException as e:
            rospy.logerr("No se pudo abrir el puerto serial: {0}".format(e))
            rospy.logerr("run: sudo chmod 666 /dev/ttyTHS1")
            raise  # Termina la ejecucion si no se puede abrir el puerto

    def __call__(self):
        if self.ser is None or self.th is None:
            rospy.logwarn("Puerto serial no inicializado o datos invalidos.")
            return

        try:
            # Validar y convertir los valores
            data = [int(val) for val in self.th]
            if all(0 <= n <= 255 for n in data):
                self.ser.write(struct.pack("BBB", *data))
                #rospy.loginfo("Enviados: {0}".format(data))
            else:
                rospy.logwarn("Valores fuera de rango (0-255): {0}".format(data))
        except Exception as e:
            rospy.logerr("Error al enviar por UART: {0}".format(e))

def callback(msg, uart_obj):
    uart_obj.th = [msg.x, msg.y, msg.z]
    uart_obj()  # Envia los datos

def main():
    rospy.init_node('UART_Node')
    try:
        serialUART = UART()
        rospy.Subscriber("reference", Point, callback, callback_args=serialUART) # joint_angle, reference

        rospy.spin()
    except Exception as e:
        rospy.logerr("Error en el nodo UART: {0}".format(e))
    finally:
        if hasattr(serialUART, 'ser') and serialUART.ser:
            serialUART.ser.close()
            rospy.loginfo("Puerto serial cerrado correctamente.")

if __name__ == '__main__':
    main()
