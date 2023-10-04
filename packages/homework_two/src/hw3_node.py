#!/usr/bin/env python3
import rospy
import math
import time
from mystery_package.msg import UnitsLabelled
from turtlesim.msg import Pose
# UnitsLabelled = float32 value, string units
# Pose = x, y, theta, linear_velocity, angular_velocity (all float32)

class MeasureDist:
    def __init__(self):
        rospy.Subscriber('turtle1/pose', Pose, self.callback)
        self.pub = rospy.Publisher('turtle1/dist_traveled', UnitsLabelled, queue_size=10)
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.start_flag = 0
        self.dist_total = 0.0
        self.dist_msg = UnitsLabelled()
        self.dist_msg.units = "meters"
    
    def callback(self, msg):
        # update current and previous position
        if self.start_flag == 0:
            self.curr_x = msg.x
            self.curr_y = msg.y
            self.start_flag = 1
        self.prev_x = self.curr_x
        self.prev_y = self.curr_y
        self.curr_x = msg.x
        self.curr_y = msg.y
        # calculate and publish distance traveled
        self.dist_total += math.sqrt( (self.curr_x - self.prev_x)**2 + \
        (self.curr_y - self.prev_y)**2 )
        #print(dist_total)
        self.dist_msg.value = self.dist_total
        self.pub.publish(self.dist_msg)
        
        
    
if __name__ == '__main__':
    rospy.init_node('measure_dist', anonymous = True)
    MeasureDist()
    rospy.spin()
        
