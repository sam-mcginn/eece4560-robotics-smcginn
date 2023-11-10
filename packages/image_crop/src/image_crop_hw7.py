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
        self.pub1 = rospy.Publisher('/image_cropped', Image, queue_size=10)
        self.pub2 = rospy.Publisher('/image_white', Image, queue_size=10)
        self.pub3 = rospy.Publisher('/image_yellow', Image, queue_size=10)
        rospy.Subscriber('/image', Image, self.crop_image)
        
        # instantiate converter object
        self.bridge = CvBridge()
        
        # Filter ranges
        self.lower_yt = numpy.array([0, 0, 245])
        self.upper_yt = numpy.array([255, 10, 255])
        self.lower_ylw = numpy.array([20, 150, 0])
        self.upper_ylw = numpy.array([32, 255, 255])
	
    def crop_image(self, image):
        # convert to a ROS image using the bridge
        self.cv_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.height = self.cv_img.shape[0]
        self.width = self.cv_img.shape[1]
        
        # 1st image - crop top 50%
        self.crop_img = self.cv_img[int(self.height/2):(self.height),0:(self.width)]
        
        # Convert colors from RGB --> HSV
        self.img_hsv = cv2.cvtColor(self.crop_img, cv2.COLOR_BGR2HSV)
        
        # 2nd image - filter for white pixels
        self.img_yt = cv2.inRange(self.img_hsv, self.lower_yt, self.upper_yt)
        
        # 3rd image - filter for yellow pixels
        self.img_ylw = cv2.inRange(self.img_hsv, self.lower_ylw, self.upper_ylw)
        
        # convert new image to ROS to send
        self.crop_msg = self.bridge.cv2_to_imgmsg(self.crop_img, "passthrough")
        self.yt_msg = self.bridge.cv2_to_imgmsg(self.img_yt, "passthrough")
        self.ylw_msg = self.bridge.cv2_to_imgmsg(self.img_ylw, "passthrough")
        
        # publish cropped images
        self.pub1.publish(self.crop_msg)
        self.pub2.publish(self.yt_msg)
        self.pub3.publish(self.ylw_msg)
	

if __name__ == '__main__':
    rospy.init_node('image_cropper', anonymous=True)
    Image_Crop()
    rospy.spin()
	
