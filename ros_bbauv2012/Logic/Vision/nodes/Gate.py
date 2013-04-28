#!/usr/bin/env python2

import roslib; roslib.load_manifest('Vision')
import rospy
import sys
import cv2 as cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from bbauv_msgs.msg import controller_input
from numpy.numarray.numerictypes import Float
from com.histogram.histogram import bbHistogram
class Gate:
    light = 0
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 48, 'hueHigh':86,'valLow':0,'valHigh':255,'Kp':10,'Vmax':40 }
    target = None
    velocity = 0
    histClass = bbHistogram()
    
    def __init__(self):
        imageTopic = rospy.get_param('~image', '/frontcam/camera/image_raw')
        compassTopic = rospy.get_param('~compass', '/os5000_data')
        self.image_pub = rospy.Publisher("/Vision/Gate/correction",Float32)
        self.image_pub2 = rospy.Publisher("/Vision/Gate/image_thres",Image)
        self.bridge = CvBridge()
        self.histClass.setParams(self.params)
        cv2.namedWindow("Gate Settings",cv2.CV_WINDOW_AUTOSIZE)
        self.image_sub = rospy.Subscriber(imageTopic, Image,self.callback)
        self.histClass.setParams(self.params)
        
        def paramSetter(key):
            def setter(val):
                self.params[key] = val
            return setter

        cv2.createTrackbar("Kp constant:", "Gate Settings", self.params['Kp'], 1000, paramSetter('Kp'));
        cv2.createTrackbar("Vmax:", "Gate Settings", self.params['Vmax'], 100, paramSetter('Vmax')); #in centimetres/second
        
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
        
    def calcCentroid(self,moments):
        return moments['mu01']/moments['mu00']
    def calcVelocity(self,pos,imgCenter):
        error = imgCenter - pos
        correction = self.params['Kp']
        return correction
    # Set up param configuration window
    def callback(self,data):
        try:
            cv_image = self.rosimg2cv(data)
            #cv2.imshow("Actual image", cv_image)   
        except CvBridgeError, e:
            print e
        #print cv_image.shape
        rows, cols, val = cv_image.shape
        cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        cv_image = np.array(cv_image,dtype=np.uint8)
        self.params = self.histClass.getParams()
        '''Perform Thresholding on HSV'''
        COLOR_MIN = np.array([self.params['hueLow'],self.params['satLow'],self.params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.params['hueHigh'],self.params['satHigh'],self.params['valHigh']],np.uint8)
        self.histClass.calcHist(cv_image)
        cv_single = cv2.inRange(cv_image,COLOR_MIN, COLOR_MAX)
        '''Perform Morphological Operations on binary image to clean it up'''
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
        cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE,kernel_close,iterations=5)
        '''Find contours on binary image and identify target to home in on'''
        contours,hierarchy = cv2.findContours(cv_single, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contourIdx = -1
        contourImg = np.zeros((240,320,3),dtype=np.uint8)
        color = cv2.cv.Scalar(100,100,100)
        centroidx = list()
        centroidy = list()
        for i in range(0,len(contours)):
            moments =cv2.moments(contours[i],binaryImage=False)
            if moments['m00'] > 400:
                cv2.drawContours(contourImg, contours, i, (100,255,100),thickness= -1)
                print len(contours)
                centroidy.append(moments['m01']/moments['m00'])
                centroidx.append(moments['m10']/moments['m00'])
                cv2.circle(contourImg,(int(centroidx[len(centroidx) -1]),int(centroidy[len(centroidy) - 1])), 2, 255, thickness=-1)
            #calculate central plane for AUV to aim towards
            if(len(centroidx) == 2):
                cv2.line(contourImg,(int(centroidx[1]),int(centroidy[1])),(int(centroidx[0]),int(centroidy[0])),(255,0,0),thickness= 1,lineType=cv2.CV_AA)
                target = (centroidx[0] + centroidx[1])/2
                self.velocity = self.calcVelocity(target, cols/2)
                cv2.circle(contourImg,(int(target),int(rows/2)), 2, (0,0,255), thickness=-1)
            else:
                self.velocity = 0
        #cv2.imshow("image window", contourImg)
        cv2.waitKey(3)
        #cv_thres = cv2.split(cv_image)
        #cv_thres = cv2.adaptiveThreshold(cv_image, 254, cv2.cv.CV_ADAPTIVE_THRESH_MEAN_C, cv2.cv.CV_THRESH_BINARY, 3, 0)
        #cv2.imshow("Image window", np.array(cv_thres,dtype=np.uint8))   
        
        #cv2.waitKey(3)
        
        try:
            cv_image = cv2.cv.fromarray(cv_image)
            cv_single = cv2.cv.fromarray(cv_single)
            contourImg = cv2.cv.fromarray(contourImg)
            if self.target != None:
                self.image_pub.publish(float(self.target))
            self.image_pub2.publish(self.bridge.cv_to_imgmsg(contourImg,encoding="bgr8"))
            #cv2.waitKey(1)
        except CvBridgeError, e:
            print e

def main(args):
    rospy.init_node('Gate', anonymous=True)
    
    gate = Gate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    

if __name__ == '__main__':
    main(sys.argv)