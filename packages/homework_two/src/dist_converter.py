#!/usr/bin/env python3
import rospy
import math
import time
from mystery_package.msg import UnitsLabelled
from std_msgs.msg import String
# UnitsLabelled = float32 value, string units

class DistanceConverter:
    def __init__(self):
        self.curr_unit = "smoots"
        rospy.Subscriber('turtle1/dist_traveled', UnitsLabelled, self.convert)
        self.pub = rospy.Publisher('turtle1/dist_converted', UnitsLabelled, queue_size=10)
    
    def convert(self, position):
        if rospy.has_param('unit'):
            self.curr_unit = rospy.get_param('unit')
        else:
            self.curr_unit = 'smoots'
    
        self.dist_msg = UnitsLabelled()
        m_to_ft = 3.2808
        m_to_smoot = 1.7018
        if self.curr_unit == 'meters':
            self.dist_msg.units = 'meters'
            self.dist_msg.value = position.value
        elif self.curr_unit == 'feet':
            self.dist_msg.units = 'feet'
            self.dist_msg.value = m_to_ft * position.value
        else:
            #smoot
            self.dist_msg.units = 'smoots'
            self.dist_msg.value = m_to_smoot * position.value
        self.pub.publish(self.dist_msg)     
  
  
    
if __name__ == '__main__':
    rospy.init_node('dist_converter', anonymous=True)
    DistanceConverter()
    rospy.spin()
