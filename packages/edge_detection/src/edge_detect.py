#!/usr/bin/env python3
import rospy
import numpy
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Image.msg = Header header, uint32 height,width,step(row length in bytes)
# string encoding, uint8 is_bigendian, uint8[] data
# HW7 topics: /image_cropped, /image_white, /image_yellow

class Edge_Detect:
    def __init__(self):
        # Cropped image --> canny edge detection
        self.sub1 = Subscriber('/image_cropped', Image)
        self.pub1 = rospy.Publisher('/image_edges', Image, queue_size=10)
        
        # White filtered image --> Hough transform
        self.sub2 = Subscriber('/image_white', Image)
        self.pub2 = rospy.Publisher('/image_lines_white', Image, queue_size=10)
        
        # Yellow filtered image --> Hough transform
        self.sub3 = Subscriber('/image_yellow', Image)
        self.pub3 = rospy.Publisher('/image_lines_yellow', Image, queue_size=10)
        
        # Synchronize all three messages to one callback
        self.ats = ApproximateTimeSynchronizer([self.sub1, self.sub2, self.sub3], queue_size=5, slop=0.1)
        self.ats.registerCallback(self.got_images)
        
    	# Converter object to convert ROS image <--> OpenCV image
        self.bridge2 = CvBridge()

    def got_images(self, img1, img2, img3):
        # Convert ROS images --> OpenCV images
        self.cv_img1 = self.bridge2.imgmsg_to_cv2(img1, "passthrough")
        self.cv_img2 = self.bridge2.imgmsg_to_cv2(img2, "passthrough")
        self.cv_img3 = self.bridge2.imgmsg_to_cv2(img3, "passthrough")
         
        # Canny edge detection on cropped image
        self.canny_cropped = cv2.Canny(self.cv_img1, 85, 255)
         
        # AND canny cropped image with white, yellow images
        self.yt_edges = cv2.bitwise_and(self.cv_img2, self.canny_cropped)
        self.ylw_edges = cv2.bitwise_and(self.cv_img3, self.canny_cropped)
         
        # Do Hough transform on both ANDed images
        # cv2.HoughLinesP( img, rho, theta, threshold, minLineLength, maxLineGap)
        self.yt_hough = cv2.HoughLinesP(self.yt_edges, 1, numpy.pi/180, 25, 25, 25)
        self.ylw_hough = cv2.HoughLinesP(self.ylw_edges, 1, numpy.pi/180, 25, 25, 25)
         
        # Add lines from Hough transform to original cropped image
        # cv2.line(img, start_pt, end_pt, BGR_color, line_thicknes)
        if self.yt_hough is not None:
            for line in self.yt_hough:
                for x1, y1, x2, y2 in line:
                    self.yt_lines = cv2.line(self.cv_img1, (x1, y1), (x2, y2), (0, 0, 255), 2)
        else:
            self.yt_lines = self.canny_cropped
                
        if self.ylw_hough is not None:
            for line in self.ylw_hough:
                for x1, y1, x2, y2 in line:
                    self.ylw_lines = cv2.line(self.cv_img1, (x1, y1), (x2, y2), (0, 0, 255), 2)
        else:
            self.ylw_lines = self.canny_cropped
                    
         
        # Publish as ROS images
        self.canny_img = self.bridge2.cv2_to_imgmsg(self.canny_cropped, "passthrough")
        self.yt_ln_img = self.bridge2.cv2_to_imgmsg(self.yt_lines, "passthrough")
        self.ylw_ln_img = self.bridge2.cv2_to_imgmsg(self.ylw_lines, "passthrough")
        self.pub1.publish(self.canny_img)
        self.pub2.publish(self.yt_ln_img)
        self.pub3.publish(self.ylw_ln_img)
    
    
if __name__ == '__main__':
    rospy.init_node('edge_detect', anonymous=True)
    Edge_Detect()
    rospy.spin()
        
        
        
