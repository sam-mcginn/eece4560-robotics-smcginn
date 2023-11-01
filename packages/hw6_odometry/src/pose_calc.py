#!usr/bin/env python3
import rospy
import roscpp
from odometry_hw import Pose2D
from odometry_hw import DistWheel
import math
# Pose2D = float64 x,y,theta
# DistWheel = float64 dist_wheel_left, dist_wheel_right

class PoseCalc:
    def __init__(self):
        self.pub=rospy.Publisher('/pose', Pose2D, queue_size=10)
        rospy.Subscriber('/dist_wheel', DistWheel, self.callback)
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0
        d_wheel = 0.1    # wheel diameter
        self.pose_msg = Pose2D()
    
    def callback(self, msg):
        # calculate change in position
        self.arc_length = (msg.dist_wheel_right + msg.dist_wheel_left)/2.0
        self.del_theta = (msg.dist_wheel_right - msg.dist_wheel_left)/self.d_wheel
        self.curr_theta = self.curr_theta + (self.del_theta/2.0)
        self.del_x = (self.arc_length)*math.cos(self.curr_theta)
        self.del_y = (self.arc_length)*math.sin(self.curr_theta)
        self.curr_x = self.curr_x + self.del_x
        self.curr_y = self.curr_y + self.del_y
        
        # publish change in pose
        self.pose_msg.x = self.curr_x
        self.pose_msg.y = self.curr_y
        self.pose_msg.theta = self.curr_theta
        self.pub.publish(self.pose_msg)
    
    
if __name__ == '__main__':
    rospy.init_node("pose_calc", anonymous=True)
    t = PoseCalc()
