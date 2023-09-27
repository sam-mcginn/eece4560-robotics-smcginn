#!/usr/bin/env python3
import rospy
import roscpp
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
import math
import time

class Talker:
    def __init__(self):
        self.pub=rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        i=1

    def talk(self):
        move_msg=Twist()
        if i==1:
            move_msg.linear.x=1.0
            move_msg.linear.y=0.0
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
            i=2
        elif i==2:
            move_msg.linear.x=0.0
            move_msg.linear.y= -1.0
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
            i=3
        elif i==3:
            move_msg.linear.x= -1.0
            move_msg.linear.y=0.0
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
            i=4
        else:
            move_msg.linear.x=0.0
            move_msg.linear.y=1.0
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
            i=1
        self.pub.publish(move_msg)
        
if __name__ == "__main__":
    try:
        rospy.init_node('hw_two_pub', anonymous=True)
        t=Talker()
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            t.talk()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass 

