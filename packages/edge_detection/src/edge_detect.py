#!/usr/bin/env python3
import rospy
import sys
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
        self.ats = ApproximateTimeSynchronizer([self.sub1, self.sub2, self.sub3], queue_size=10, slop=0.5)
        self.ats.registerCallback(self.got_images)
        
    	# Converter object to convert ROS image <--> OpenCV image
        self.bridge2 = CvBridge()

    def got_images(self, img1, img2, img3):
        # Convert ROS images --> OpenCV images
        cv_img1 = self.bridge2.imgmsg_to_cv2(img1, "bgr8")
        cv_img2 = self.bridge2.imgmsg_to_cv2(img2, "mono8")
        cv_img3 = self.bridge2.imgmsg_to_cv2(img3, "mono8")
         
        # Canny edge detection on cropped image (100,200)
        canny_cropped = cv2.Canny(cv_img1, 175, 250)
        canny_img = self.bridge2.cv2_to_imgmsg(canny_cropped, "mono8")
        self.pub1.publish(canny_img)
        
        # Dilate yellow, white images to capture edges better
        bloat_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (12,12))
        cv_img2 = cv2.dilate(cv_img2, bloat_kernel)
        cv_img3 = cv2.dilate(cv_img3, bloat_kernel)
        #self.cv_img2 = cv2.GaussianBlur(self.cv_img2, (3,3))
        #self.cv_img2 = cv2.GaussianBlur(self.cv_img2, (3,3))
         
        # AND canny cropped image with white, yellow images
        self.yt_edges = cv2.bitwise_and(cv_img2, canny_cropped)
        self.ylw_edges = cv2.bitwise_and(cv_img3, canny_cropped)
        
        rho1 = 1
        theta1 = numpy.pi/180.0
        threshold1 = 10
        # lines1 = None
        minLineLength1 = 10
        maxLineGap1 = 10
        
         
        # Do Hough transform on both ANDed images
        # cv2.HoughLinesP( img, rho, theta, threshold, lines(?) minLineLength, maxLineGap)
        self.yt_hough = cv2.HoughLinesP(self.yt_edges, rho1, theta1, threshold1, minLineLength=minLineLength1, maxLineGap=maxLineGap1)
        rospy.loginfo("Found: "+ str(self.yt_hough))
        self.ylw_hough = cv2.HoughLinesP(self.ylw_edges, rho1, theta1, threshold1, minLineLength=minLineLength1, maxLineGap=maxLineGap1)
        rospy.loginfo("Found: "+ str(self.ylw_hough))
        
        # Add lines from Hough transform to original cropped image
        # cv2.line(img, start_pt, end_pt, BGR_color, line_thickness)

        if self.yt_hough is not None:
            yt_lines = self.output_lines(cv_img1, self.yt_hough)
        else:
            # If no lines found in Hough transform - show img that it scanned
            self.yt_edges = cv2.cvtColor(self.yt_edges, COLOR_GRAY2BGR)
            yt_lines = self.yt_edges
                
        if self.ylw_hough is not None:
            ylw_lines = self.output_lines(cv_img1, self.ylw_hough)
        else:
            # If no lines found in Hough transform - show img that it scanned
            self.ylw_edges = cv2.cvtColor(self.ylw_edges, COLOR_GRAY2BGR)
            ylw_lines = self.ylw_edges

        
        '''
        # cv2.HoughLines(img, rho, theta, threshold)
        self.yt_hough = cv2.HoughLines(self.yt_edges, 1, numpy.pi/180, 5)
        self.ylw_hough = cv2.HoughLines(self.ylw_edges, 1, numpy.pi/180, 5)
        
        if self.yt_hough is not None:
            self.yt_lines = self.cv_img1
            for line in self.yt_hough:
                rospy.loginfo("Found line: "+ str(line))
                for rho, theta in line:
                    a = numpy.cos(theta)
                    b = numpy.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(x0 - 1000*(a))
                    self.yt_lines = cv2.line(self.yt_lines, (x1, y1), (x2, y2), (0, 0, 255), 2)
        else:
            self.yt_lines = self.yt_edges
        
        if self.ylw_hough is not None:
            self.ylw_lines = self.cv_img1
            for line in self.ylw_hough:
                rospy.loginfo("Found line: "+ str(line))
                for rho, theta in line:
                    a = numpy.cos(theta)
                    b = numpy.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(x0 - 1000*(a))
                    self.ylw_lines = cv2.line(self.ylw_lines, (x1, y1), (x2, y2), (0, 0, 255), 2)
        else:
            self.ylw_lines = self.ylw_edges
        '''
        
                    
         
        # Publish as ROS images
        yt_ln_img = self.bridge2.cv2_to_imgmsg(yt_lines, "bgr8")
        ylw_ln_img = self.bridge2.cv2_to_imgmsg(ylw_lines, "bgr8")
        
        self.pub2.publish(yt_ln_img)
        self.pub3.publish(ylw_ln_img)
        
        
    def output_lines(self, original_image, lines):
        # Takes in an image and array of line vectors, returns the image with the lines drawn on
        output = numpy.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output
    
    
if __name__ == '__main__':
    rospy.init_node('edge_detect', anonymous=True)
    Edge_Detect()
    rospy.spin()
        
        
        
