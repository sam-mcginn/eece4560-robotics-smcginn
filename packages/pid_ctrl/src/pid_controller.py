#!/usr/bin/env python3
import rospy
import math
from std_msgs import Float32

class PID_Ctrl:
    def __init__(self):
        # Gain parameters
        self.kp = 0;
        self.ki = 0;
        self.kd = 0;
        
        self.prev_error;
        self.error_total;
        
        # Publish to controller, subscribe to error
        self.pub = rospy.Publisher('control_input', Float32, queue_size=10)
        rospy.Subscriber('error', Float32, self.callback_pid)
        
    def callback_pid(self, error):
        dt = 1
        
        # Calculate proportional
        proportional = self.kp*error
        
        # Calculate integral
        # FIX
        self.error_total = self.error_total + error
        integral = (self.ki)*(self.error_total)*dt
        
        
        # Calculate derivative
        # FIX
        derivative = (self.kd)*(error - self.prev_error)/(dt)
        
        gain = Float32()
        gain = proportional + integral + derivative
        self.pub.publish(gain)
        
if __name__ == '__main__':
    rospy.init_node('sensor_transform', anonymous=True)
    PID_Ctrl()
    rospy.spin()



