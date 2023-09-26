#!/usr/bin/env python
import rospy
import roscpp
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
import math
import time

def hw_two_pub():
    pub = rospy.Publisher('turtlesim/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('hw_two_pub', anonymous=True)
    rate = rospy.Rate(1)
    if not rospy.is_shutdown():
        move_msg = Twist()
        move_msg.linear.x = 1.0
        move_msg.linear.y = 1.0
        move_msg.linear.z = 0.0
        move_msg.angular.x = 0.0
        move_msg.angular.y = 0.0
        move_msg.angular.z = 0.0
        pub.publish(move_msg)
        
if __name__ == "__main__":
    hw_two_pub()  

