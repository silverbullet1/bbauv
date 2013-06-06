#!/usr/bin/env python2

import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib


import smach
import smach_ros
from smach import StateMachine
from smach_ros import SimpleActionState
import PID_Controller.msg
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import sys
import cv2 as cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from numpy.numarray.numerictypes import Float
from com.histogram.histogram import bbHistogram
from numpy.ma.core import fabs
from actionlib_msgs.msg._GoalStatus import GoalStatus

'''
###################################################################

                       COMPUTER VISION CLASS 
        
###################################################################

'''

class Gate:
    light = 0
    target = 0
    rows = 0
    col = 0
    velocity = 0
    params = { 'satLow': 50, 'satHigh': 255, 'hueLow': 13, 'hueHigh':38,'valLow':0,'valHigh':255,'Kp':10,'Vmax':40 }
    client = None;
    histClass = None
    centroidx = list()
    centroidy = list()
    bridge = None
     # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
    
    def __init__(self):
        self.bridge = CvBridge()
        #imageTopic = rospy.get_param('~image', '/stereo_camera/left/image_color')
        imageTopic = rospy.get_param('~image', '/stereo_camera/left/image_rect_color_remote')
        cv2.namedWindow("Gate Settings",cv2.CV_WINDOW_AUTOSIZE)
        image_sub = rospy.Subscriber(imageTopic, Image,self.computeImageCallback)
        self.image_pub2 = rospy.Publisher("/Vision/image_filter",Image)
        self.histClass = bbHistogram()
        self.histClass.setParams(self.params)
        def paramSetter(key):
            def setter(val):
                self.params[key] = val
            return setter
        cv2.createTrackbar("Kp constant:", "Gate Settings", self.params['Kp'], 1000, paramSetter('Kp'));
        cv2.createTrackbar("Vmax:", "Gate Settings", self.params['Vmax'], 100, paramSetter('Vmax')); #in m/second
    
    def calcCentroid(self,moments):
        return moments['mu01']/moments['mu00']
    def calcVelocity(self,pos,imgCenter):
        error = imgCenter - pos
        correction = self.params['Kp']
        return correction
       
    def computeImageCallback(self,data):
        if(self.bridge!= None):
            try:
                cv_image = self.rosimg2cv(data)
            #cv2.imshow("Actual image", cv_image)   
            except CvBridgeError, e:
                print e
            self.rows, self.cols, val = cv_image.shape
            cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
            cv_image = np.array(cv_image,dtype=np.uint8)
            if self.histClass != None:   
                self.params = self.histClass.getParams()
            '''Perform Thresholding on HSV'''
            COLOR_MIN = np.array([self.params['hueLow'],self.params['satLow'],self.params['valLow']],np.uint8)
            COLOR_MAX = np.array([self.params['hueHigh'],self.params['satHigh'],self.params['valHigh']],np.uint8)
            self.histClass.getTripleHist(cv_image)
            cv_single = cv2.inRange(cv_image,COLOR_MIN, COLOR_MAX)
            '''Perform Morphological Operations on binary image to clean it up'''
            kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
            kernel_close = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
            cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
            cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE,kernel_close,iterations=5)
            '''Find contours on binary image and identify target to home in on'''
            contours,hierarchy = cv2.findContours(cv_single, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contourIdx = -1
            contourImg = np.zeros((480,640,3),dtype=np.uint8)
            color = cv2.cv.Scalar(100,100,100)
            centroidx_temp = list()
            centroidy_temp = list()
            centroidx_hack = list()
            centroidy_hack = list()
            for i in range(0,len(contours)):
                moments =cv2.moments(contours[i],binaryImage=False)
                if moments['m00'] > 400:
                    cv2.drawContours(contourImg, contours, i, (100,255,100),thickness= -1)
                    #print len(contours)
                    centroidy_temp.append(moments['m01']/moments['m00'])
                    centroidx_temp.append(moments['m10']/moments['m00'])
                    cv2.circle(contourImg,(int(centroidx_temp[len(centroidx_temp) -1]),int(centroidy_temp[len(centroidy_temp) - 1])), 2, 255, thickness=-1)
                #calculate central plane for AUV to aim towards
            
            if(len(centroidx_temp) > 1):
                centroidx_hack.append(centroidx_temp[0])
                centroidy_hack.append(centroidy_temp[0])
                ctr = 1
                
                while(len(centroidx_temp) -1 > ctr):
                    if(fabs(centroidx_hack[0] - centroidx_temp[ctr]) >50):
                        centroidx_hack.append(centroidx_temp[ctr])
                        centroidy_hack.append(centroidy_temp[ctr])
                        break    
                    ctr = ctr + 1
                if(len(centroidx_hack) == 2):
                    cv2.line(contourImg,(int(centroidx_hack[1]),int(centroidy_hack[1])),(int(centroidx_hack[0]),int(centroidy_hack[0])),(255,0,0),thickness= 1,lineType=cv2.CV_AA)
                    self.target = (centroidx_hack[0] + centroidx_hack[1])/2
                    cv2.circle(contourImg,(int(self.target),int(self.rows/2)), 2, (0,0,255), thickness=-1)
                    
            cv2.waitKey(3)
            try:
                contourImg = cv2.cv.fromarray(contourImg)
                if self.target != None:
                    pass
                    #self.image_pub.publish(float(self.target))
                self.image_pub2.publish(self.bridge.cv_to_imgmsg(contourImg,encoding="bgr8"))
                #cv2.waitKey(1)
            except CvBridgeError, e:
                print e
            except ROSException, e:
                rospy.logwarn("Topic closed during node shutdown" + str(e))
            if(len(centroidx_hack) == 2):
                self.centroidx = centroidx_hack
                self.centroidy = centroidy_hack
                
'''
###################################################################

               SMACH STATE MACHINE CLASS DECLARATION
        
###################################################################

'''            
                
class Search(smach.State):
    global gate
    global mission_srv_request
    global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete','aborted'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing state SEARCH')
        while(len(gate.centroidx) != 2 and not rospy.is_shutdown()):
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

class MotionControlProcess(smach.State):
    global gate
    global r
    isCorrection = False
    Kp = 0.001
    client = None
    sidemove = 0
    def __init__(self):
        smach.State.__init__(self, outcomes=['task_complete','aborted'],
                             output_keys=['complete_output'],
                             )
    def done_cb(self,status,result):
        #global movement_client
        global locomotionGoal
        rospy.loginfo("action sent from callback")
        if status==actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("callback goal sent")
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=2,heading_setpoint=locomotionGoal.heading_setpoint,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=self.sidemove)
            self.client.send_goal(goal,self.done_cb)
        
    def execute(self,userdata):
        global locomotionGoal
        #global movement_client
        #rospy.loginfo('Executing state VISION_PROCESSING')
        self.client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        self.client.wait_for_server()
        
        self.client.cancel_all_goals()
        
        while(not rospy.is_shutdown()):
            if(len(gate.centroidx) == 2):
               target = (gate.centroidx[0] + gate.centroidx[1])/2 - 640*0.5
               if(len(gate.centroidx)==2):
                   centroid_dist = fabs(gate.centroidx[0] - gate.centroidx[1])
               if(target>0 and fabs(target) >10):
                   self.sidemove = centroid_dist*self.Kp
               elif(target<0 and fabs(target) >10):
                   self.sidemove = -centroid_dist*self.Kp
               else:
                   self.sidemove = 0
                   self.client.cancel_all_goals()
                   locomotionGoal.sidemove_setpoint = 0
                   userdata.complete_output = True
                   try:
                       resp = mission_srv_request(False,True,locomotionGoal)
                   except rospy.ServiceException, e:
                       print "Service call failed: %s"%e
                   return 'task_complete'
               rospy.logdebug("target:" + str(target) + "sidemove:" + str(self.sidemove))
               if(not self.isCorrection):
                   rospy.loginfo("vision action issued!")
                   goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=2,heading_setpoint=locomotionGoal.heading_setpoint,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=self.sidemove)
                   self.client.send_goal(goal,self.done_cb)
                   self.isCorrection = True
            #r.sleep()
        return 'aborted'
        

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
        #Service Server
        self.client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        self.client.wait_for_server()
        
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
    
'''
###################################################################

                       MAIN PYTHON THREAD
        
###################################################################

'''
x = 100
gate = None
image = None
r = None
isAbort = False
isStart = False
isEnd = False
locomotionGoal = controller()
mission_srv_request = None
vision_srv = None
movement_client = None
if __name__ == '__main__':
    rospy.init_node('Gate', anonymous=True)
    r = rospy.Rate(30)
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    vision_srv = rospy.Service('gate_srv', mission_to_vision, handle_srv)
    rospy.loginfo('gate_srv initialized!')
    
    #Service Client
    rospy.loginfo('waiting for mission_srv...')
    rospy.wait_for_service('mission_srv')
    mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission)
    rospy.loginfo('connected to mission_srv!')
    #resp = mission_srv(ctrl,None)
    
    gate = Gate()
    #Initialize State Machine
    sm_top = smach.StateMachine(outcomes=['gate_complete','aborted'])
    #Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED',
                         Disengage(),
                         transitions={'start_complete':'SEARCH','complete_outcome':'gate_complete','aborted':'aborted'},
                         remapping={'complete_input':'goal'}
                         )
        smach.StateMachine.add('SEARCH',
                         Search(),
                         transitions={'search_complete':'MOTION_CONTROL','aborted':'aborted'})
        # Create the sub SMACH state machine for Motion Control Phase 
        #sm_motion = smach.StateMachine(['task_complete'])
        
        
        smach.StateMachine.add('MOTION_CONTROL',
                         MotionControlProcess(),
                         transitions={'task_complete': 'DISENGAGED','aborted':'aborted'},
                         remapping={'complete_output':'goal'}
                         )
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('gate_server', sm_top, '/MISSION/GATE')
    sis.start()
    sm_top.userdata.goal = False
   # gate = Gate()
    try:
        outcome = sm_top.execute()
        rospy.spin()
    except KeyboardInterrupt:
        sis.stop()
        print "Shutting down"
        
        
        
        
       
        
    