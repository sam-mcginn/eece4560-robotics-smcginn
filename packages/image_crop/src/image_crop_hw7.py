#!/usr/bin/env python3
import rospy
import numpy
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# Image.msg = Header header, uint32 height,width,step(row length in bytes)
# string encoding, uint8 is_bigendian, uint8[] data

class Image_Crop:
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
        height = cv_img.shape[0]
        width = cv_img.shape[1]
        
        # 1st image - crop top 50%
        cv_img1 = cv_img[(height/2):(height),0:width]
        
        # 2nd image - filter for white pixels
        
        # 3rd image - filter for 
        
        # convert new image to ROS to send
        
        # publish cropped images
        self.crop_msg = Image()
        self.white_msg = Image()
        seelf.yellow_msg = Image()
	
	

if __name__ == '__main__':
    rospy.init_node('image_cropper', anonymous=True)
    Image_Crop()
    rospy.spin()
	
