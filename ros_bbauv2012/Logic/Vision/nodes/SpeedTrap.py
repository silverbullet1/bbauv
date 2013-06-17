#!/usr/bin/env python2

'''
Created on Apr 25, 2013

@author: gohew
'''
#ROS and System libraries
import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib

#User libraries
from com.histogram.histogram import Hist_constants
from com.histogram.histogram import bbHistogram
from com.shape.ShapeAnalysis import ShapeAnalysis
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *

from nav_msgs.msg import Odometry

import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import sys
import smach
import smach_ros
import math
import os

#Dynamic Reconfigure

from dynamic_reconfigure.server import Server
from Vision.cfg import SpeedTrapConfig

#External libraries
import numpy as np

'''
###################################################################

                       COMPUTER VISION CLASS
        
###################################################################
'''

class SpeedTrap:
    debug = True
    red_params = {'hueLow': 0, 'hueHigh':10,'satLow': 100, 'satHigh': 255,'valLow':0,'valHigh':255,'topHueLow':170,'topHueHigh':180}
    yellow_params = {'hueLow': 20, 'hueHigh':60,'satLow': 0, 'satHigh': 255,'valLow':0,'valHigh':255}
    stParams = {'canny': 135 }    
    yellow_hist = bbHistogram("yellow",Hist_constants.TRIPLE_CHANNEL)
    red_hist = bbHistogram("red",Hist_constants.TRIPLE_CHANNEL)
    isAlignState = True
    isLoweringState = True
    isAim = False
    isCentering = False
    #shapeClass = ShapeAnalysis()
    yaw = 0
    centroidx = 0
    centroidy = 0
    rows = 0
    cols = 0
    binList = None
    orientation = None
    #params = { 'satLow': 50, 'satHigh': 255, 'hueLow': 36, 'hueHigh':48,'valLow':0,'valHigh':255,'Kp':10,'Vmax':40 }
    bridge = None
    position = None
    centroidx_list = None
    centroidy_list = None
    max_area = 0
    outer_center = 40
    inner_center = 20
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
    Node Functions
    '''    
    def __init__(self):
        #imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_rect_color')
        #yawTopic = rospy.get_param('~compass', '/euler')
        self.bridge = CvBridge()
        self.yellow_hist.setParams(self.yellow_params)
        self.red_hist.setParams(self.red_params)
        #cv2.namedWindow("Sub Alignment",cv2.CV_WINDOW_AUTOSIZE)
        #cv2.moveWindow("Sub Alignment",512,30)
        self.image_pub = rospy.Publisher("/Vision/image_filter",Image)
        self.image_sub = rospy.Subscriber('/bottomcam/camera/image_rect_color_remote', Image,self.processImage)
        self.yaw_sub = rospy.Subscriber('/euler',compass_data,self.collectYaw)
        self.pos_sub = rospy.Subscriber('/WH_DVL_data',Odometry)
        self.bridge = CvBridge()
    
    def collectPosition(self,msg):
        self.position = (msg.pose.pose.position.x , msg.pose.pose.position.y)
    def collectYaw(self,msg):
        self.yaw = msg.yaw
    
    def aiming(self,image):
        if self.debug:
            self.red_hist.setParams(self.red_params)
        COLOR_MIN = np.array([self.red_params['hueLow'],self.red_params['satLow'],self.red_params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.red_params['hueHigh'],self.red_params['satHigh'],self.red_params['valHigh']],np.uint8)
        
        bot_red_image = cv2.inRange(image,COLOR_MIN, COLOR_MAX)
        
        COLOR_MIN = np.array([self.red_params['topHueLow'],self.red_params['satLow'],self.red_params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.red_params['topHueHigh'],self.red_params['satHigh'],self.red_params['valHigh']],np.uint8)
        
        top_red_image = cv2.inRange(image,COLOR_MIN, COLOR_MAX)
        
        red_image = top_red_image + bot_red_image
        
        if self.debug:
            self.red_hist.getTripleHist(image)
        '''Find contours on binary image and identify target to home in on'''
        '''Perform Morphological Operations on binary image to clean it up'''
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE,kernel_close,iterations=5)
        if(self.isLoweringState):
            shape_array = cv2.split(image)
            #retval, shape_image = cv2.threshold(shape_array[2], self.params['valLow'], 255, cv2.THRESH_OTSU)
            #self.histClass.setTrackBarPosition("Value Low:", int(retval))
            #retval, red_image = cv2.threshold(shape_array[0], self.red_params['hueLow'], 255, cv2.THRESH_OTSU)
            #self.red_hist.setTrackBarPosition("red_Hue Low:", int(retval))
            contours,hierarchy = cv2.findContours(red_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            contourImg = np.zeros((self.rows,self.cols,3),dtype=np.uint8)
            if(contours != None):
                for i in range(0,len(contours)):
                    moments =cv2.moments(contours[i],binaryImage=False)
                    if moments['m00'] > 50:
                        humoments = cv2.HuMoments(moments)
                        cv2.drawContours(contourImg, contours, i, (0,0,255), lineType=8, thickness= 1,maxLevel=0)
            #shape_image = shape_array[2] - white_image
            #shape_image1 = np.copy(shape_image)
            #self.shapeClass.predictSurf(shape_array[2])
        return contourImg
    
    def centroidIdentification(self,image):
        
        '''
        
            PERFORM YELLOW THRESHOLDING ON IMAGE
        
        '''
        #self.yellow_param = self.yellow_hist.getParams()
        if self.debug:
            self.yellow_hist.setParams(self.yellow_params)
        #COLOR_MIN = np.array([self.yellow_params['hueLow'],self.yellow_params['satLow'],self.yellow_params['valLow']],np.uint8)
        #COLOR_MAX = np.array([self.yellow_params['hueHigh'],self.yellow_params['satHigh'],self.yellow_params['valHigh']],np.uint8)
        imgArray = cv2.split(image)
        
        #retval, cv_thres = cv2.threshold(imgArray[0], self.yellow_params['hueLow'], 255, cv2.THRESH_OTSU)
        #self.yellow_hist.setTrackBarPosition("yellow_Hue Low:", int(retval))
        retval = 91
        if retval > 90:
            hue_low = self.yellow_params['hueLow']
            hue_high = self.yellow_params['hueHigh']
        else:
            hue_low = 0
            hue_high = 0
        
        COLOR_MIN = np.array([hue_low,self.yellow_params['satLow'],self.yellow_params['valLow']],np.uint8)
        COLOR_MAX = np.array([hue_high,self.yellow_params['satHigh'],self.yellow_params['valHigh']],np.uint8)
        cv_single = cv2.inRange(image,COLOR_MIN, COLOR_MAX)
        
        if self.debug:
            self.yellow_hist.getTripleHist(image)
        if retval < 90:
            cv_single = cv_single + cv_thres
            
        '''Find contours on binary image and identify target to home in on'''
        '''Perform Morphological Operations on binary image to clean it up'''
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
        cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE,kernel_close,iterations=1)
        contours,hierarchy = cv2.findContours(cv_single, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.rows,self.cols,val = image.shape
        contourImg = np.zeros((self.rows,self.cols,3),dtype=np.uint8)
        centroidx = list()
        centroidy = list()
        binList = list()
        angleList = list()
        if(contours != None):
            for i in range(0,len(contours)):
                moments =cv2.moments(contours[i],binaryImage=False)
                if moments['m00'] > 300:
                    humoments = cv2.HuMoments(moments)
                    cv2.drawContours(contourImg, contours, i, (100,255,100), lineType=8, thickness= 1,maxLevel=0)
                    contourRect= cv2.minAreaRect(contours[i])
                    pos,size,theta = contourRect
                    binList.append(contourRect)
                    #cv2.rectangle(contourImg, (rect[0],rect[1]), (rect[0] + rect[2],rect[1] + rect[3]), (255,0,0))
                    # Obtain the actual corners of the box
                    points = cv2.cv.BoxPoints(contourRect)
                    longest_pt1 = None
                    longest_pt2 = None
                    longest_norm = 0
                    # Draw the lines
                    for i in range(4):
                        # The line function doesn't accept floating point values
                        pt1 = (int(points[i][0]), int(points[i][1]))
                        pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                        norm = math.sqrt(pow((pt1[0] - pt2[0]),2) + pow((pt1[1] - pt2[1]),2))
                        if (norm > longest_norm):
                            longest = i
                            longest_norm = norm
                            longest_pt1 = pt1
                            longest_pt2 = pt2
                        cv2.line(contourImg, pt1, pt2, 255, 1)
                    if(abs(humoments[0] - 0.202) < 0.01):
                        if(longest_pt2[1] < longest_pt1[1]):
                            temp = longest_pt2
                            longest_pt2 = longest_pt1
                            longest_pt1 = temp 
                        rect_y = longest_pt2[1] - longest_pt1[1]
                        rect_x = longest_pt2[0] - longest_pt1[0]
                        angle_hor = math.degrees(math.atan2(rect_y,(rect_x))) 
                        #print angle_hor
                        angleList.append(angle_hor)
                        cv2.putText(contourImg,str(np.round(angle_hor,1)), (int(points[0][0]),int(points[0][1])), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
                    if True:
                        colorTxt = (0,0,255)
                        #cv2.drawContours(contourImg, contours, i, (255,0,0),thickness= -1)
                        if(self.isAlignState):
                            centroidy.append(moments['m01']/moments['m00'])
                            centroidx.append(moments['m10']/moments['m00'])
                            cv2.circle(contourImg,(int(centroidx[len(centroidx) -1]),int(centroidy[len(centroidy) - 1])), 2, (0,0,255), thickness=-1)
                            #calculate central plane for AUV to aim towards
                        if(len(centroidx) > 3):
                            for j in range(0,len(centroidx) -1):
                                pass
                                #'''cv2.line(contourImg,(int(centroidx[j]),int(centroidy[j])),(int(centroidx[j+1]),int(centroidy[j+1])),(255,255,0),thickness= 1,lineType=cv2.CV_AA)
                                #targetAngle = math.degrees(math.atan((centroidy[j] -centroidy[j+1])/(centroidx[j] -centroidx[j+1])))
                                #targetAngle = math.degrees(math.atan((centroidy[j] -centroidy[j+1]),(centroidx[j] -centroidx[j+1])))
                                #cv2.putText(contourImg,str(np.round(targetAngle,1)), (int(centroidx[j]),int(centroidy[j])), cv2.FONT_HERSHEY_PLAIN, 1, colorTxt)
                                #self.computeCorrection(targetAngle)
                                #target = (centroidx[0] + centroidx[1])/2
                                #cv2.circle(contourImg,(int(target),int(rows/2)), 2, (0,0,255), thickness=-1)
                    else:
                        colorTxt = (0,255,255)
                    #cv2.putText(contourImg,"u0:" + str(np.round(humoments[0][0],3)),(int(points[0][0]),int(points[0][1])), cv2.FONT_HERSHEY_PLAIN, 1, colorTxt)
        else:
            self.centroidx = 0
            self.centroidy = 0
        # Compute Central centroid for vehicle centering
        if len(centroidx) > 0:
            self.centroidx_list = centroidx
            self.centroidy_list = centroidy
            self.centroidx, self.centroidy = self.computeCenter(centroidx, centroidy) 
            cv2.circle(contourImg,(int(self.centroidx),int(self.centroidy)), 2, (0,0,255), thickness=-1)
        
        #Compute orientation angle correction for vehicle orientation
        if len(angleList) >3:
            self.isCentering = True
            self.orientation = np.average(angleList, None, None)
        else:
            self.isCentering = False
        
        #Compute Max area of all bounding rectangles
        #pos,size,theta
        max_area = 0
        if len(binList) > 0:
            for rect in binList:
                temp_area = int(rect[1][0])*int(rect[1][1])
                if temp_area > max_area:
                    max_area = temp_area
            self.max_area = max_area
        #Display orientation angles
        if self.orientation != None:
            cv2.putText(contourImg,str(np.round(self.orientation,1)) + " " + str(np.round(self.yaw,1)), (int(self.centroidx),int(self.centroidy)), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        return contourImg
    
    def computeCenter(self,centroid_x,centroid_y):
       x_ave = np.average(centroid_x,None,None)
       y_ave = np.average(centroid_y,None,None)
       return x_ave,y_ave
   
    def processImage(self,data): 
        try:
            cv_image = self.rosimg2cv(data)
        except CvBridgeError, e:
            print e
        #cv_single = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        #cv_single = np.array(cv_single,dtype=np.uint8)
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        hsv_image = np.array(hsv_image,dtype=np.uint8)
        centroid_image = self.centroidIdentification(hsv_image)
        #Draw aiming window on image to center
        
        cv2.rectangle(centroid_image,(300,220),(340,260), (255,0,0))
        if self.isAim:
            shape_image = self.aiming(hsv_image)
        else:
            shape_image = np.zeros((self.rows,self.cols,3),dtype=np.uint8)
        final_image = shape_image + centroid_image
        cv2.waitKey(1)
        try:
            if(final_image != None):
                final_image= cv2.cv.fromarray(final_image)
                #cv_single = cv2.cv.fromarray(cv_single)
                #contourImg = cv2.cv.fromarray(contourImg)
                if(self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(final_image,encoding="bgr8"))
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
                            input_keys=['complete'])
    def execute(self,userdata):
        global locomotionGoal
        global isStart
        global isEnd
        global mission_srv_request
        
        if userdata.complete == True:
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
       global r
       global st
       while(st.centroidx == 0 and not rospy.is_shutdown()):
           r.sleep()
       if rospy.is_shutdown():
           return 'aborted'
       else:
        '''
        try:
            resp = mission_srv_request(True,False,None)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''
        return 'search_complete'
       

class Centering(smach.State):
    global mission_srv_request
   # global r
    K = 0.005
    def __init__(self):
        smach.State.__init__(self, outcomes=['centering_complete','aborted'],output_keys=['center_pos'])
        
    def execute(self,userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        isOrientationDone = False
        center_complete = False
        while(not center_complete and not rospy.is_shutdown()):
            if(st.centroidx != 0):
                side_error = self.K*(st.centroidx - st.cols/2)
                fwd_error = -self.K*(st.centroidy - st.rows/2)
                if(st.orientation != None):
                    if st.isCentering and isOrientationDone == False:
                        if(st.orientation > 90):
                            orientation_error = (st.yaw - (180 - st.orientation)) % 360
                        else:
                            orientation_error = (st.yaw - st.orientation) % 360
                        rospy.loginfo("box_orient:" + str(st.orientation) + "yaw:" + str(st.yaw) +  "final yaw:" + str(orientation_error))
                        isOrientationDone = True
                else:
                    orientation_error = locomotionGoal.heading_setpoint
                if(np.fabs(st.centroidx - st.cols/2) < st.outer_center and np.fabs(st.centroidy - st.rows/2) <st.outer_center and np.fabs(orientation_error - st.yaw) <5):
                    #userdata.center_pos = st.position
                    locomotionGoal.heading_setpoint = orientation_error
                    movement_client.cancel_all_goals()
                    return "centering_complete"
                print "orientation error:" + str(orientation_error) + "isCentering:" + str(st.isCentering)
                goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=fwd_error,heading_setpoint=orientation_error,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=side_error)
                movement_client.send_goal(goal)
                movement_client.wait_for_result(rospy.Duration(1))
                rospy.loginfo("Centering...")
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            '''
            try:
                resp = mission_srv_request(True,False,None)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            '''
            return 'search_complete'
       
class Orientating(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['orientating_complete','aborted'],input_keys=['center_pos'])
        
    def execute(self,userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        orientation_complete = False
        #while(not center_complete and not rospy.is_shutdown()):
        #userdata.center_pos[0]
        #goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=correction,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=0)
        #movement_client.send_goal(goal)
        #movement_client.wait_for_result()
        rospy.loginfo("turn achieved!")
        #r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            return 'orientating_complete' 
        
class Aiming(smach.State):
    global st
    global mission_srv_request
    K = 0.005
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['aiming_complete','aborted'],input_keys=['center_pos'])
        
    def execute(self,userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        isLowering = True
        depth_offset = 0
        while(len(st.centroidx_list) > 0 and not rospy.is_shutdown()):
            aim_x = np.min(st.centroidx_list, None, None)
            aim_y = np.min(st.centroidy_list, None, None)
            print st.max_area
            side_error = self.K*(aim_x - st.cols/2)
            fwd_error = -self.K*(aim_y - st.rows/2)
            if ((np.fabs(aim_x - st.cols/2) <st.inner_center and np.fabs(aim_y - st.rows/2) <st.inner_center) and st.max_area > 1500 ) or (depth_offset + locomotionGoal.depth_setpoint) > 3 :
                st.isAim = True
                isLowering = False
                rospy.loginfo("Identifying target...")
                #movement_client.cancel_all_goals()
                #return "aiming_complete"
            if isLowering:
                depth_offset = depth_offset + 0.2
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=fwd_error,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint + depth_offset,
                                                     sidemove_setpoint=side_error)
            movement_client.send_goal(goal)
            movement_client.wait_for_result(rospy.Duration(1))
            rospy.loginfo("Centering and Lowering...")
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
        
def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    rospy.loginfo("Gate service handled.")
    if req.start_request:
        rospy.loginfo("isStart true.")
        isStart = True
        # Format for service: start_response, abort_response
        locomotionGoal = req.start_ctrl
    if req.abort_request:
        isAbort = True
    return mission_to_visionResponse(isStart,isAbort)

#Global Variables

movement_client = None
locomotionGoal = None 
isStart = True
isAbort = False  
isEnd = False
st = None     
r = None
if __name__ == '__main__':
    rospy.init_node('SpeedTrap', anonymous=False)
    r = rospy.Rate(20)
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    locomotionGoal = bbauv_msgs.msg.ControllerGoal()
    locomotionGoal.heading_setpoint = 130
    locomotionGoal.depth_setpoint = 0.6
    vision_srv = rospy.Service('speedtrap_srv', mission_to_vision, handle_srv)
    rospy.loginfo('speedtrap_srv initialized!')
    
    #Service Client
    rospy.loginfo('waiting for mission_srv...')
    #rospy.wait_for_service('mission_srv')
    #mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission)
    rospy.loginfo('connected to mission_srv!')
    st = SpeedTrap()
    
    # Set up param configuration window
    def configCallback(config, level):
        for param in st.yellow_params:
            st.yellow_params[param] = config['yellow_' + param]
        for param in st.red_params:
            st.red_params[param] = config['red_'+param]
        return config
    srv = Server(SpeedTrapConfig, configCallback)
   
    sm_top = smach.StateMachine(outcomes=['speedtrap_complete','aborted'])
    #Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED',
                         Disengage(),
                         transitions={'start_complete':'SEARCH','complete_outcome':'speedtrap_complete','aborted':'aborted'}
                         )
        smach.StateMachine.add('SEARCH',
                         Search(),
                         transitions={'search_complete':'CENTERING','aborted':'aborted'})
        smach.StateMachine.add('CENTERING',
                         Centering(),
                         transitions={'centering_complete': 'AIMING','aborted':'aborted'}
                         )
        smach.StateMachine.add('AIMING',
                         Aiming(),
                         transitions={'aiming_complete': 'DISENGAGED','aborted':'aborted'}
                         )

    sis = smach_ros.IntrospectionServer('server',sm_top,'/MISSION/SPEEDTRAP')
    sis.start()
    sm_top.userdata.complete = False
    # Execute SMACH plan
    outcome = sm_top.execute()
    try:
        rospy.spin()
        #sis.stop()
    except KeyboardInterrupt:
        print "Shutting down"
    pass