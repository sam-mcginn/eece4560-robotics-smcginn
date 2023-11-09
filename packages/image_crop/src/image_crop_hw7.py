#!/usr/bin/env python3
import rospy
import numpy
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

Class Image_Crop:
    def __init__(self):
        # set up publishers and subscribers
        self.pub = rospy.Publisher('/image_cropped', Image, queue_size=10)
        self.pub = rospy.Publisher('/image_white', Image, queue_size=10)
        self.pub = rospy.Publisher('/image_yellow', Image, queue_size=10)
        rospy.Subscriber('/image', Image, self.crop_image)
        
        # instantiate converter object
        self.bridge = CvBridge()
	
    def crop_image(self, image):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        height = cv_img.shape(0)
        
        # 1st image - crop top 50%
        cv_img1 = cv_img[((height/2):(height),:)
        
        # 2nd image - filter for white pixels
        
        # 3rd image - filter for 
        
        # convert new image to ROS to send
        
        # publish cropped images
	
	

if __name__ == '__main__':
    rospy.init_node('image_cropper', anonymous=True)
    Image_Crop()
    rospy.spin()
	
