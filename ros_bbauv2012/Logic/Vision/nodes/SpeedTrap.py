'''
Created on Apr 25, 2013

@author: gohew
'''
#!/usr/bin/env python2

#User libraries
from com.histogram.histogram import Hist_constants
from com.histogram.histogram import bbHistogram
from com.shape.ShapeAnalysis import ShapeAnalysis

#ROS and System libraries
import roslib; roslib.load_manifest('Vision')
import rospy
import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from bbauv_msgs.msg import controller_input
import sys
import smach
import smach_ros
import math
import os

#External libraries
import numpy as np

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        print 'Executing state FOO'
        if self.counter < 3:
            self.counter += 1
            rospy.loginfo("executing outcome 1!")
            return 'outcome1'
        else:
            return 'outcome2'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        print 'Executing state BAR'
        return 'outcome2'

class SpeedTrap:
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 0, 'hueHigh':255,'valLow':0,'valHigh':255,'grayLow':0 ,'grayHigh':255}
    stParams = {'canny': 135 }    
    histClass = bbHistogram(Hist_constants.DUAL_CHANNEL_MODE)
    isAlignState = True
    isLoweringState = True
    shapeClass = ShapeAnalysis()
    ''' 
    Utility Methods
    '''
    
    def paramSetter(self,key):
        def setter(val):
            self.params[key] = val
        return setter    
    
    def stParamSetter(self,key):
        def setter(val):
            self.stParams[key] = val
        return setter    
  
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image trackbarnameto Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
    
    def maskROI(self,image,rect):
        maskImage = np.copy(image)
        row, col= maskImage.shape
        x = rect[0]
        y = rect[1]
        maskImage[0:y] = 0
        maskImage[0:x] = 0
        maskImage[y + rect[3]:row] = 0
        maskImage[x+ rect[2]:col] = 0
        return maskImage
    ''' 
    Vehicle Action Methods
    '''
    
    def computeCorrection(self,angle):
        print "c"
    
    '''
    Node Functions
    '''    
    def __init__(self):
        imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_raw')
        compassTopic = rospy.get_param('~compass', '/os5000_data')
        self.image_pub = rospy.Publisher("/Vision/SpeedTrap/image_filter",Image)
        self.bridge = CvBridge()
        self.histClass.setParams(self.params)
        cv2.namedWindow("Sub Alignment",cv2.CV_WINDOW_AUTOSIZE)
        cv2.moveWindow("Sub Alignment",512,30)
        cv2.createTrackbar("Canny Threshold:", "Sub Alignment", self.stParams['canny'], 500, self.stParamSetter('canny'));
        self.image_sub = rospy.Subscriber(imageTopic, Image,self.processImage)
        self.bridge = CvBridge()
               
    def processImage(self,data):
        try:
            cv_image = self.rosimg2cv(data)
        except CvBridgeError, e:
            print e
        cv_single = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        cv_single = np.array(cv_single,dtype=np.uint8)
        shape_image = None
        #hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        #hsv_image = np.array(hsv_image,dtype=np.uint8)
        self.params = self.histClass.getParams()
        COLOR_MIN = np.array([self.params['hueLow'],self.params['satLow'],self.params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.params['hueHigh'],self.params['satHigh'],self.params['valHigh']],np.uint8)
        self.histClass.getTripleHist(cv_image)
        self.histClass.getSingleHist(cv_single)
        retval, cv_single = cv2.threshold(cv_single, self.params['grayLow'], 255, cv2.THRESH_OTSU)
        self.histClass.setTrackBarPosition("Gray Low:", int(retval))
        #cv_single = cv2.adaptiveThreshold(cv_single, 254, cv2.cv.CV_ADAPTIVE_THRESH_MEAN_C, cv2.cv.CV_THRESH_BINARY_INV, 5, 7)
        #cv_single = cv2.GaussianBlur(cv_single, (3,3),1)
        #cv_single = cv2.Canny(cv_single,self.stParams['canny'], self.stParams['canny']*3,L2gradient=True)
        '''Find contours on binary image and identify target to home in on'''
        '''Perform Morphological Operations on binary image to clean it up'''
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
        cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE,kernel_close,iterations=5)
        contours,hierarchy = cv2.findContours(cv_single, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        contourImg = np.zeros((240,320,3),dtype=np.uint8)
        centroidx = list()
        centroidy = list()
        rectContourList = list()
        for i in range(0,len(contours)):
            moments =cv2.moments(contours[i],binaryImage=False)
            if moments['m00'] > 300 and moments['m00'] < 10000:
                humoments = cv2.HuMoments(moments)
                cv2.drawContours(contourImg, contours, i, (100,255,100),thickness= -1)
                rect = cv2.boundingRect(contours[i])
                cv2.rectangle(contourImg, (rect[0],rect[1]), (rect[0] + rect[2],rect[1] + rect[3]), (255,0,0))
                #identified as rectangle
                if abs(humoments[0] - 0.205) < 0.005:
                    rectContourList.append(i)
                    colorTxt = (0,0,255)
                    cv2.drawContours(contourImg, contours, i, (255,0,0),thickness= -1)
                    if(self.isAlignState):
                        centroidy.append(moments['m01']/moments['m00'])
                        centroidx.append(moments['m10']/moments['m00'])
                        cv2.circle(contourImg,(int(centroidx[len(centroidx) -1]),int(centroidy[len(centroidy) - 1])), 2, (0,0,255), thickness=-1)
                        #calculate central plane for AUV to aim towards
                    if(self.isLoweringState):
                        shape_array = cv2.split(cv_image)
                        #retval, shape_image = cv2.threshold(shape_array[2], self.params['valLow'], 255, cv2.THRESH_OTSU)
                        self.histClass.setTrackBarPosition("Value Low:", int(retval))
                        retval, white_image = cv2.threshold(shape_array[0], self.params['valLow'], 255, cv2.THRESH_OTSU)
                        self.histClass.setTrackBarPosition("Hue Low:", int(retval))
                        shape_image = shape_array[2] - white_image
                        shape_image1 = np.copy(shape_image)
                        self.shapeClass.predictSurf(shape_array[2])
                        #final_image = self.findShapeTarget(shape_image1)
                    if(len(centroidx) > 1):
                        for j in range(0,len(centroidx) -1):
                            cv2.line(contourImg,(int(centroidx[j]),int(centroidy[j])),(int(centroidx[j+1]),int(centroidy[j+1])),(255,255,0),thickness= 1,lineType=cv2.CV_AA)
                            #targetAngle = math.degrees(math.atan((centroidy[j] -centroidy[j+1])/(centroidx[j] -centroidx[j+1])))
                            targetAngle = math.degrees(math.atan2((centroidy[j] -centroidy[j+1]),(centroidx[j] -centroidx[j+1])))
                            cv2.putText(contourImg,str(np.round(targetAngle,1)), (int(centroidx[j]),int(centroidy[j])), cv2.FONT_HERSHEY_PLAIN, 1, colorTxt)
                            self.computeCorrection(targetAngle)
                            #target = (centroidx[0] + centroidx[1])/2
                            #cv2.circle(contourImg,(int(target),int(rows/2)), 2, (0,0,255), thickness=-1)
                else:
                    colorTxt = (100,100,100)
                cv2.putText(contourImg,"u0:" + str(np.round(humoments[0][0],3)), (contours[i][0][0][0],contours[i][0][0][1]), cv2.FONT_HERSHEY_PLAIN, 0.8, colorTxt)
        
        #cv_corners = cv2.cornerHarris(cv_single, 5, 3,0.5)
        #cv_corners = np.array(cv_corners,dtype=np.uint8)
        
        cv2.imshow("Sub Alignment", contourImg)
        cv2.waitKey(1)
        try:
            if(shape_image != None):
                cv_canny = cv2.cv.fromarray(shape_image)
                #cv_single = cv2.cv.fromarray(cv_single)
                #contourImg = cv2.cv.fromarray(contourImg)
                self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_canny))
            #cv2.waitKey(1)
        except CvBridgeError, e:
            print e
        
        
if __name__ == '__main__':
    rospy.init_node('SpeedTrap', anonymous=True)
    st = SpeedTrap()
    s = smach.StateMachine(outcomes=['outcome4'])
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome2':'FOO'})

    sis = smach_ros.IntrospectionServer('server',sm,'/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    try:
        rospy.spin()
        #sis.stop()
    except KeyboardInterrupt:
        print "Shutting down"
    pass