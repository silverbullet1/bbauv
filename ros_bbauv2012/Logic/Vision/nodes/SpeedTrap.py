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
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import sys
import smach
import smach_ros
import math
import os

#External libraries
import numpy as np

'''
###################################################################

                       COMPUTER VISION CLASS
        
###################################################################
'''

class SpeedTrap:
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 0, 'hueHigh':255,'valLow':0,'valHigh':255,'grayLow':0 ,'grayHigh':255}
    stParams = {'canny': 135 }    
    histClass = bbHistogram(Hist_constants.DUAL_CHANNEL_MODE)
    isAlignState = True
    isLoweringState = True
    shapeClass = ShapeAnalysis()
    yaw = 0
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
        imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_rect_color')
        yawTopic = rospy.get_param('~compass', '/euler')
        self.image_pub = rospy.Publisher("/Vision/image_filter",Image)
        self.bridge = CvBridge()
        self.histClass.setParams(self.params)
        cv2.namedWindow("Sub Alignment",cv2.CV_WINDOW_AUTOSIZE)
        cv2.moveWindow("Sub Alignment",512,30)
        cv2.createTrackbar("Canny Threshold:", "Sub Alignment", self.stParams['canny'], 500, self.stParamSetter('canny'));
        self.image_sub = rospy.Subscriber(imageTopic, Image,self.processImage)
        self.yaw_sub = rospy.Subscriber(yawTopic,bbauv_msgs.compass,self.collectYaw)
        self.bridge = CvBridge()
    
    def collectYaw(self,msg):
        self.yaw = msg.yaw
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
        #cv_single = cv2.GaussianBlur(cv3),1)
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
'''
###################################################################

               SMACH STATE MACHINE CLASS DECLARATION
        
###################################################################

'''
class Disengage(smach.State):
    client = None
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete','complete_outcome','aborted'],
                            input_keys=['complete_input'])
    def execute(self,userdata):
        global locomotionGoal
        global isStart
        global isEnd
        global mission_srv_request
        
        if userdata.complete_input == True:
             isStart = False
             isEnd = True
             try:
                 resp = mission_srv_request(False,True,locomotionGoal)
             except rospy.ServiceException, e:
                 print "Service call failed: %s"%e
        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Deactivating Gate Node")
                return 'complete_outcome'
            if isStart:
                return 'start_complete'
        return 'aborted'
    
class Search(smach.State):
    global st
    global mission_srv_request
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete','aborted'])
        
    def execute(self,userdata):
        while(len(st.centroidx) != 2 and not rospy.is_shutdown()):
            #print len(gate.centroidx)
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            try:
                resp = mission_srv_request(True,False,None)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            return 'search_complete'

class Centering(smach.State):
    global st
    global mission_srv_request
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['centering_complete','aborted'])
        
    def execute(self,userdata):
        while(len(st.centroidx) != 2 and not rospy.is_shutdown()):
            #print len(gate.centroidx)
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            try:
                resp = mission_srv_request(True,False,None)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            return 'search_complete'
       
class Orientating(smach.State):
    global st
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['orientating_complete','aborted'])
        
    def execute(self,userdata):
        while(len(st.centroidx) != 2 and not rospy.is_shutdown()):
            #print len(gate.centroidx)
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            try:
                resp = mission_srv_request(True,False,None)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            return 'search_complete' 
        
class Aiming(smach.State):
    global st
    global mission_srv_request
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['aiming_complete','aborted'])
        
    def execute(self,userdata):
        while(len(st.centroidx) != 2 and not rospy.is_shutdown()):
            #print len(gate.centroidx)
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            try:
                resp = mission_srv_request(True,False,None)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            return 'search_complete'
'''
###################################################################

                       MAIN PYTHON THREAD
        
###################################################################

'''
#Global Variables

movement_client = None
locomotionGoal = None   
     
if __name__ == '__main__':
    rospy.init_node('SpeedTrap', anonymous=True)
    
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    
    vision_srv = rospy.Service('speedtrap_srv', mission_to_vision, handle_srv)
    rospy.loginfo('speedtrap_srv initialized!')
    
    #Service Client
    rospy.loginfo('waiting for mission_srv...')
    rospy.wait_for_service('mission_srv')
    mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission)
    rospy.loginfo('connected to mission_srv!')
    
    st = SpeedTrap()
    
    sm_top = smach.StateMachine(outcomes=['speedtrap_complete','aborted'])
    #Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED',
                         Disengage(),
                         transitions={'start_complete':'SEARCH','complete_outcome':'speedtrap_complete','aborted':'aborted'},
                         )
        smach.StateMachine.add('SEARCH',
                         Search(),
                         transitions={'search_complete':'CENTERING','aborted':'aborted'})
        # Create the sub SMACH state machine for Motion Control Phase 
        #sm_motion = smach.StateMachine(['task_complete'])
        smach.StateMachine.add('CENTERING',
                         Centering(),
                         transitions={'centering_complete': 'ORIENTATING','aborted':'aborted'},
                         )
        smach.StateMachine.add('ORIENTATING',
                         Orientating(),
                         transitions={'orientating_complete': 'AIMING','aborted':'aborted'},
                         )
        smach.StateMachine.add('AIMING',
                         Aiming(),
                         transitions={'aiming_complete': 'DISENGAGED','aborted':'aborted'},
                         )

    sis = smach_ros.IntrospectionServer('server',sm_top,'/MISSION/SPEEDTRAP')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    try:
        rospy.spin()
        #sis.stop()
    except KeyboardInterrupt:
        print "Shutting down"
    pass