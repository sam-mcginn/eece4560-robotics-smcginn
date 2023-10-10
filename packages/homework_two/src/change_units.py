#!/usr/bin/env python3
import rospy
import math
import time
from mystery_package.msg import UnitsLabelled
from std_msgs.msg import String
# UnitsLabelled = float32 value, string units

Class ChangeUnits:
    def __init__(self):
        self.unit = 'smoots'
        self.pub = rospy.Publisher('turtle1/conversion_unit', String, queue_size=10)
    
    def changeUnit(self):
        if self.unit == 'smoots':
            self.unit = 'meters'
        elif self.unit == 'meters':
            self.unit = 'feet'
        else:
            # feet
            self.unit = 'smoots'
        self.pub.publish(self.unit)
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('change_units', anonymous=True)
        t=ChangeUnits()
        rate=rospy.Rate(0.2)
        while not rospy.is_shutdown():
            t.changeUnit()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    
