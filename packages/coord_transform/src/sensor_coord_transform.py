#!/usr/bin/env python3
import rospy
import numpy
import math
from duckietown_msgs.msg import Vector2D
# Vector2D: float32 x,y

class SensorTransform:
    def __init__(self):
        rospy.Subscriber('/sensor_coord', Vector2D, self.transform)
        self.pub_rc = rospy.Publisher('/robot_coord', Vector2D, queue_size=10)
        self.pub_wc = rospy.Publisher('/world_coord', Vector2D, queue_size=10)
        self.curr_scoord = numpy.matrix([[0],[0],[1]])
        self.curr_rcoord = numpy.matrix([[0],[0],[1]])
        self.curr_wcoord = numpy.matrix([[0],[0],[1]])
        self.wts_xform = numpy.matrix([[0.866, 0.5, 6.866], [-0.5, 0.866, 1.5], [0.0,0.0,1.0]])
        self.rts_xform = numpy.matrix([[-1.0,0.0,-1.0], [0.0,-1.0,0.0], [0,0,1]])
        
    def transform(self, sensor_vector):
        # initialize msgs
        self.rc_msg = Vector2D()
        self.wc_msg = Vector2D()
        
        # convert
        self.curr_scoord = numpy.matrix([[sensor_vector.x], [sensor_vector.y], [1]])
        self.curr_rcoord = self.rts_xform*self.curr_scoord
        self.curr_wcoord = self.wts_xform*self.curr_scoord
        
        # update msgs
        self.rc_msg.x = self.curr_rcoord[0,0]
        self.rc_msg.y = self.curr_rcoord[1,0]
        self.wc_msg.x = self.curr_wcoord[0,0]
        self.wc_msg.y = self.curr_wcoord[1,0]
        
        # publish
        rospy.loginfo("Robot transform: "+str(self.curr_rcoord[0,0])+str(self.curr_rcoord[1,0]));
        rospy.loginfo("World transform: "+str(self.curr_wcoord[0,0])+str(self.curr_wcoord[1,0]));
        self.pub_rc.publish(self.rc_msg)
        self.pub_wc.publish(self.wc_msg)
        
        
if __name__ == '__main__':
    rospy.init_node('sensor_transform', anonymous=True)
    SensorTransform()
    rospy.spin()
