#!/usr/bin/env python3
import rospy
import math
import time
from std_msgs.msg import Float32

class PID_Ctrl:
    def __init__(self):
        # Gain parameters
        self.kp = 0.25
        self.ki = 0
        self.kd = 0
        
        self.prev_error = 0
        self.prev_time = rospy.get_time()
        self.error_total = 0
        
        # Publish to controller, subscribe to error
        self.pub = rospy.Publisher('control_input', Float32, queue_size=10)
        rospy.Subscriber('error', Float32, self.callback_pid)
        
    def callback_pid(self, msg):
        dt = rospy.get_time() - self.prev_time
        error = msg.data
        
        # Calculate proportional
        proportional = self.kp*error
        
        # Calculate integral
        self.error_total = self.error_total + error
        integral = (self.ki)*(self.error_total)*dt
        
        
        # Calculate derivative
        derivative = (self.kd)*(error - self.prev_error)/(dt)
        
        # Update prev_error, prev_time
        self.prev_error = error
        self.prev_time = rospy.get_time()
        
        gain = Float32()
        gain = proportional + integral + derivative
        self.pub.publish(gain)
        
if __name__ == '__main__':
    rospy.init_node('sensor_transform', anonymous=True)
    PID_Ctrl()
    rospy.spin()



