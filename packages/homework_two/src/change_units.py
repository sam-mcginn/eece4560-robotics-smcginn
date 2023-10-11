#!/usr/bin/env python3
import rospy
import math
import time
from mystery_package.msg import UnitsLabelled
from std_msgs.msg import String
# UnitsLabelled = float32 value, string units

class ChangeUnits:
    def __init__(self):
        self.unit = 'smoots'
    
    def changeUnit(self):
        if self.unit == 'smoots':
            self.unit = 'meters'
        elif self.unit == 'meters':
            self.unit = 'feet'
        else:
            # feet
            self.unit = 'smoots'
        
        if rospy.has_param('unit'):
            rospy.set_param('unit', self.unit)
        else:
            # FIX - add error log?
    
    
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
    
