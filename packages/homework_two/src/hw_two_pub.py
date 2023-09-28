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

    def talk(self, i):
        move_msg=Twist()
        if i==1:
            move_msg.linear.x=2.1
            move_msg.linear.y=0.0
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
        elif i==2:
            move_msg.linear.x=0.0
            move_msg.linear.y= -2.1
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
        elif i==3:
            move_msg.linear.x= -2.1
            move_msg.linear.y=0.0
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
        else:
            move_msg.linear.x=0.0
            move_msg.linear.y=2.1
            move_msg.linear.z=0.0
            move_msg.angular.x=0.0
            move_msg.angular.y=0.0
            move_msg.angular.z=0.0
        self.pub.publish(move_msg)
        
if __name__ == "__main__":
    try:
        rospy.init_node('hw_two_pub', anonymous=True)
        t=Talker()
        rate=rospy.Rate(1)
        num=1
        while not rospy.is_shutdown():
            t.talk(num)
            if num<4:
            	num+=1
            else:
                num=1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass 

