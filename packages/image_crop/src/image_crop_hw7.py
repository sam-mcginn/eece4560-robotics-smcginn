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
	
    def crop_image(self, image):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        height = cv_img.shape[0]
        width = cv_img.shape[1]
        
        # 1st image - crop top 50%
        crop_img = cv_img[int(height/2):(height),0:width]
        
        # Convert colors from RGB --> HSV
        img_hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # 2nd image - filter for white pixels
        lower_yt = numpy.array([0, 0, 0])
        upper_yt = numpy.array([0, 10, 255])
        img_yt = cv2.inRange(img_hsv, lower_yt, upper_yt)
        
        # 3rd image - filter for yellow pixels
        lower_ylw = numpy.array([22, 100, 20])
        upper_ylw = numpy.array([37, 255, 255])
        img_ylw = cv2.inRange(img_hsv, lower_ylw, upper_ylw)
        
        # convert new image to ROS to send
        crop_msg = self.bridge.cv2_to_imgmsg(crop_img, desired_encoding='passthrough')
        yt_msg = self.bridge.cv2_to_imgmsg(img_yt, desired_encoding='passthrough')
        ylw_msg = self.bridge.cv2_to_imgmsg(img_ylw, desired_encoding='passthrough')
        
        # publish cropped images
        self.pub1.publish(self.crop_msg)
        self.pub2.publish(self.yt_msg)
        self.pub3.publish(self.ylw_msg)
	

if __name__ == '__main__':
    rospy.init_node('image_cropper', anonymous=True)
    Image_Crop()
    rospy.spin()
	
