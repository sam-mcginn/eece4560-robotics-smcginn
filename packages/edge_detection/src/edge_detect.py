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
        ats = ApproximateTimeSynchronizer([self.sub1, self.sub2, self.sub3], queue_size=5, slop=0.1)
        ats.registerCallback(self.got_images)
        
    	# Converter object to convert ROS image <--> OpenCV image
        self.bridge2 = CvBridge()

    def got_images(self, img1, img2, img3):
        # Convert ROS images --> OpenCV images
        self.cv_img1 = self.bridge2.imgmsg_to_cv2(img1, "bgr8")
        self.cv_img2 = self.bridge2.imgmsg_to_cv2(img2, "bgr8")
        self.cv_img3 = self.bridge2.imgmsg_to_cv2(img3, "bgr8")
         
        # Canny edge detection on cropped image
        self.canny_cropped = cv2.Canny(self.cv_img1, 50, 150)
         
        # AND canny cropped image with white, yellow images
         
        # Do Hough transform on both ANDed images
         
        # Add lines from Hough transform to original cropped image
         
        # Publish as ROS images
        self.canny_img = self.bridge2.cv2_to_imgmsg(self.canny_cropped, "passthrough")
        self.pub1.publish(self.canny_img)
 
    '''
    def do_canny(self, image):
        # Convert ROS image --> OpenCV image:
        self.cv_img1 = self.bridge.imgmsg_to_cv2(image, "bgr8")
        # Canny edge detection:
        self.canny_cv = cv2.Canny(self.cv_img1, 50, 150)
        # Convert OpenCV image --> ROS image:
        self.canny_img = self.bridge.cv2_to_imgmsg(self.canny_cv, "passthrough")
        # Publish ROS image:
        #self.pub1.publish(self.canny_img)
        
    def hough_white(self, image):
        # Convert ROS image --> OpenCV image:
        self.cv_img2 = self.bridge.imgmsg_to_cv2(image, "bgr8")
        # Hough transform:
        
        # Convert OpenCV image --> ROS image
        # Publish ROS image:
    
    def hough_yellow(self, image):
        # Convert ROS image --> OpenCV image:
        # Hough transform:
        # Convert OpenCV image --> ROS image
        # Publish ROS image:
        pass
    '''
    
    
if __name__ == '__main__':
    rospy.init_node('edge_detect', anonymous=True)
    Edge_Detect()
    rospy.spin()
        
        
        
