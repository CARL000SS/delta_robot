#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
import time


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0
        self.measure = 0
        self.sample_time = 0.01
        self.output_limits = (0, 100)
        
        self._last_time = None
        self._last_input = None
        self._integral = 0
        self._last_output = None

        self.output = 0
    
    def control(self, dt=None):
        """
        Calcula la salida del controlador PID para el valor de entrada dado.
        """
        now = time.time()
        
        if dt is None:
            dt = now - self._last_time if self._last_time is not None else self.sample_time
        elif dt <= 0:
            rospy.loginfo('dt tiene un valor negativo o cero')
            return
        
        if self._last_time is None:
            dt = self.sample_time
        
        error = self.setpoint - self.measure
        
        proportional = self.Kp * error
        
        self._integral += self.Ki * error * dt

        if self.output_limits[0] is not None and self.output_limits[1] is not None:
            self._integral = max(self.output_limits[0], min(self.output_limits[1], self._integral))
        
        derivative = 0
        if self._last_input is not None and dt > 0:
            derivative = self.Kd * (self.measure - self._last_input) / dt
        
        self.output = proportional + self._integral - derivative
        
        if self.output_limits[0] is not None and self.output_limits[1] is not None:
            self.output = max(self.output_limits[0], min(self.output_limits[1], self.output))
        
        self._last_input = self.measure
        self._last_time = now
        self._last_output = self.output
    
    def tune(self, Kp, Ki, Kd):
        """Ajusta las ganancias del controlador"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

def callback_position(msg):
    Xpid.measure = msg.x
    Ypid.measure = msg.y
    Hpid.measure = msg.z

def callback_reference(msg):
    Xpid.setpoint = msg.x
    Ypid.setpoint = msg.y
    Hpid.setpoint = msg.z
    
def loop():
    while not rospy.is_shutdown():

        Xpid.control()
        Ypid.control()
        Hpid.control()

        msg = Point()
        msg.x = Xpid.output
        msg.y = Ypid.output
        msg.z = Hpid.output

        pub.publish(msg)
        rate.sleep()
    



def main():
    try:
        loop()
        rospy.spin()
    except Exception as error: print(error)
    except rospy.ROSInterruptException:
        print("Node Terminated!")
    

if __name__ == '__main__':

    rospy.init_node('control')
    rospy.Subscriber("position_measure", Point, callback_position)
    rospy.Subscriber("reference", Point, callback_reference)
    pub = rospy.Publisher('platform_position', Point, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    Xpid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
    Ypid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
    Hpid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)

    main()

