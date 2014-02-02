#!/usr/bin/env python

import roslib; roslib.load_manifest('vision')

from bbauv_msgs.msg import * 
from sensor_msgs.msg import Image

import rospy
import cv2 
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class BucketDetector:
    red_params = {'lowerH' : 0, 'lowerS' : 0, 'lowerV' : 100, 'higherH' : 77, 'higherS' : 195, 'higherV' : 251 }
    areaThresh = 50000

    bridge = None
    
    curHeading = 0
    angleList = None
    max_area = 0
    
    screen = {}
    screen['width'] = 640
    screen['height'] = 480
        
    '''
    Utility Methods 
    '''
    #Convert ROS image to Numpy matrix for cv2 functions 
    def rosimg2cv(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)

        return frame 

    '''
    Bucket Node vision methods
    '''
    def __init__(self):
        self.rectData = { 'detected' : False }

        self.image_topic = rospy.get_param('~image', '/bot_cam/camera/image_rect_color_opt')
        self.image_pub = rospy.Publisher("/Vision/image_filter_opt_bucket" , Image)
        self.register()

        #TODO: Add histogram modes for debug
        self.bridge = CvBridge()
        rospy.loginfo("Bucket ready")
            
    def register(self):
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.processImage)
        self.headingSub = rospy.Subscriber('/euler', compass_data, self.compassCallback)
        rospy.loginfo("Topics registered")
        
    def unregister(self):
        self.image_sub.unregister()
        self.curHeading.unregister()
        rospy.loginfo("Topics unregistered")
    
    def compassCallback(self, data):
        self.curHeading = data.yaw
    
    #Perform red thresholding
    def findTheBucket(self, image):
        #Convert ROS to CV image 
        cv_image = self.rosimg2cv(image)
        
        cv_image = cv2.GaussianBlur(cv_image, ksize=(5, 5), sigmaX=0)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #Convert to HSV image

        #Perform yellow thresholding
        lowerBound = np.array([self.red_params['lowerH'],
                               self.red_params['lowerS'],
                               self.red_params['lowerV']], 
                               np.uint8)
        higherBound = np.array([self.red_params['higherH'],
                                self.red_params['higherS'],
                                self.red_params['higherV']],
                                np.uint8)
        contourImg = cv2.inRange(hsv_image, lowerBound, higherBound)
        
        #Noise removal
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        contourImg = cv2.erode(contourImg, erodeEl)
        contourImg = cv2.dilate(contourImg, dilateEl)

        out = cvtColor(contourImg, cv2.cv.CV_GRAY2BGR)
      
        #Find centroid
        pImg = contourImg.copy()
        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        maxArea = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.areaThresh and area > maxArea:
                maxArea = area

                # Find center with moments
                mu = cv2.moments(contour, False)
                mu_area = mu['m00']
                centroidx = mu['m10']/mu_area
                centroidy = mu['m01']/mu_area
                
                self.rectData['area'] = area
                self.rectData['centroid'] = (centroidx, centroidy)
                self.rectData['rect'] = cv2.minAreaRect(contour)
    
        #Find the largest rect area
        if maxArea > 0: 
            self.rectData['detected'] = True
            points = np.array(cv2.cv.BoxPoints(self.rectData['rect']))
            
            #Testing
            centerx = int(self.rectData['centroids'][0])
            centery = int(self.rectData['centroids'][1])
            contourImg = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2RGB)
            cv2.circle(out, (centerx, centery), 5, (0, 255, 0))
            for i in range (4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                cv2.line(contourImg, pt1, pt2, (255,0,0))
                cv2.line(out, pt1, pt2, (0, 0, 255))

            cv2.putText(out, str(self.rectData['angle']), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
            
        else:
            self.rectData['detected'] = False 
              
        return out 
              
    def computeCenter(self, centroid_x, centroid_y):
          x_ave = np.ave(centroid_x, None, None)
          y_ave = np.ave(centroid_y, None, None)
          return x_ave, y_ave
          
    def processImage(self, data):
        cv_image = self.rosimg2cv(data)
        
        centroid_image = self.findTheBucket(cv_image)
        
        try:
            if centroid_image:
                self.image_pub.publish(self.bridge.cv_to_imgmsg(centroid_image, encoding="bgr8"))
        except CvBridgeError, e:
            rospy.logerr(e) 

if __name__ == "__main__":
    rospy.init_node("bucket_vision")
    bucketDector = BucketDetector()
    rospy.spin()
