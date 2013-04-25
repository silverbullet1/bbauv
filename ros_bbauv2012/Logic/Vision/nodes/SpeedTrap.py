'''
Created on Apr 25, 2013

@author: gohew
'''
#!/usr/bin/env python2

import roslib; roslib.load_manifest('Vision')
import rospy
import sys
import cv2 as cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from bbauv_msgs.msg import controller_input
from com.histogram.histogram import bbHistogram

class SpeedTrap:
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 48, 'hueHigh':86,'valLow':0,'valHigh':255}    
    histClass = bbHistogram()
    
    def __init__(self):
        imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_raw')
        compassTopic = rospy.get_param('~compass', '/os5000_data')
        self.image_pub = rospy.Publisher("/Vision/Gate/image_filter",Image)
        self.bridge = CvBridge()
        self.histClass.setParams(self.params)
        cv2.namedWindow("SpeedTrap Settings",cv2.CV_WINDOW_AUTOSIZE)
        self.image_sub = rospy.Subscriber(imageTopic, Image,self.processImage)
        self.bridge = CvBridge()
        
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
    
    def paramSetter(self,key):
        def setter(val):
            self.params[key] = val
        return setter

    def processImage(self,data):
        try:
            cv_image = self.rosimg2cv(data)
        except CvBridgeError, e:
            print e
        cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        cv_image = np.array(cv_image,dtype=np.uint8)
        self.params = self.histClass.getParams()
        COLOR_MIN = np.array([self.params['hueLow'],self.params['satLow'],self.params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.params['hueHigh'],self.params['satHigh'],self.params['valHigh']],np.uint8)
        self.histClass.calcHist(cv_image)
        cv2.waitKey(1)
        try:
            cv_image = cv2.cv.fromarray(cv_image)
            #cv_single = cv2.cv.fromarray(cv_single)
            #contourImg = cv2.cv.fromarray(contourImg)
            self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image,encoding="bgr8"))
            #cv2.waitKey(1)
        except CvBridgeError, e:
            print e
        
        
if __name__ == '__main__':
    rospy.init_node('SpeedTrap', anonymous=True)
    
    st = SpeedTrap()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    pass