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
from bbauv_msgs.msg import controller_input
from numpy.numarray.numerictypes import Float
from com.histogram.histogram import bbHistogram
from bbauv_msgs.msg._controller import controller

class Gate:
    light = 0
    target = 0
    rows = 0
    col = 0
    velocity = 0
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 10, 'hueHigh':43,'valLow':0,'valHigh':255,'Kp':10,'Vmax':40 }
    client = None;
    centroidx = list();
    centroidy = list();
    bridge = None
     # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
    
    
    def __init__(self):
        imageTopic = rospy.get_param('~image', '/stereo_camera/left/image_color')
        cv2.namedWindow("Gate Settings",cv2.CV_WINDOW_AUTOSIZE)
        image_sub = rospy.Subscriber(imageTopic, Image,self.computeImageCallback)
        bridge = CvBridge()
        self.image_pub2 = rospy.Publisher("/Vision/Gate/image_thres",Image)
        histClass = bbHistogram()
        histClass.setParams(self.params)
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
        try:
            cv_image = self.rosimg2cv(data)
        #cv2.imshow("Actual image", cv_image)   
        except CvBridgeError, e:
            print e

        self.rows, self.cols, val = cv_image.shape
        cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        cv_image = np.array(cv_image,dtype=np.uint8)
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
                self.target = (centroidx[0] + centroidx[1])/2
                #self.velocity = self.calcVelocity(target, cols/2)
                cv2.circle(contourImg,(int(self.target),int(self.rows/2)), 2, (0,0,255), thickness=-1)
        cv2.waitKey(3)
        #cv_thres = cv2.split(cv_image)
        #cv_thres = cv2.adaptiveThreshold(cv_image, 254, cv2.cv.CV_ADAPTIVE_THRESH_MEAN_C, cv2.cv.CV_THRESH_BINARY, 3, 0)
        #cv2.imshow("Image window", np.array(cv_thres,dtype=np.uint8))   
        #cv2.waitKey(3)
        
        try:
            contourImg = cv2.cv.fromarray(contourImg)
            if self.target != None:
                self.image_pub.publish(float(self.target))
            self.image_pub2.publish(self.bridge.cv_to_imgmsg(contourImg,encoding="bgr8"))
            #cv2.waitKey(1)
        except CvBridgeError, e:
            print e
            
class Search(smach.State):
    global gate
    global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing state SEARCH')
        while(len(gate.centroidx) != 2 or not rospy.is_shutdown()):
            r.sleep()
        if rospy.is_shutdown:
            print "shutting down!"
            rospy.signal_shutdown()
        return 'search_complete'

class MotionControlProcess(smach.State):
    global gate
    global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['vision_processed','sub_task_complete'],
                             output_keys=['locomotion_out'],
                             input_keys=['movement_success_in'])
    def execute(self,userdata):
        rospy.loginfo('Executing state VISION_PROCESSING')
        while(len(gate.centroidx) != 2 or not rospy.is_shutdown()):
            r.sleep()
            
        goal = bbauv_msgs.msg.ControllerAction
        goal.depth_setpoint = 0
        goal.forward_setpoint = 1
            #goal.heading_setpoint = 
        return 'sub_task_complete'

class Disengage(smach.State):
    isStart = False
    isEnd = False
    global locomotionGoal
    def handle_srv(self,req):
        
        if req.start_request:
            self.isStart = True
            # Format for service: start_response, abort_response
            locomotionGoal.depth_setpoint = req.start_ctrl.depth_setpoint
        
        if req.abort_request:
            isAbort = False
        
        return mission_to_visionResponse(self.isStart,False)
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete','complete_outcome'])
    def execute(self,userdata):
        #Service Server
        if self.preempt_requested():
            print "pre-empt!"
        srvServer = rospy.Service('gate_srv', mission_to_vision, self.handle_srv)
        rospy.loginfo('gate_srv initialized!')
        
        #Service Client
        rospy.wait_for_service('mission_srv')
        mission_srv = rospy.ServiceProxy('mission_srv', mission_to_vision)
        rospy.loginfo('connected to mission_srv!')
        #resp = mission_srv(ctrl,None)
        
        while(not rospy.is_shutdown()):
            if self.isStart:
                return 'start_complete'
            if self.isEnd:
                return 'complete_outcome'

'''
    GLOBAL VARIABLES

'''
x = 100
gate = None
image = None
r = None
isAbort = False
locomotionGoal = controller()

if __name__ == '__main__':
    rospy.init_node('Gate', anonymous=True)
    r = rospy.Rate(20)
    gate = Gate()
    #Initialize State Machine
    rospy.loginfo('Executing SM')
    sm_top = smach.StateMachine(outcomes=['gate_complete'])
    #Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED',
                         Disengage(),
                         transitions={'start_complete':'SEARCH','complete_outcome':'gate_complete'}
                         )
        smach.StateMachine.add('SEARCH',
                         Search(),
                         transitions={'search_complete':'MOTION_CONTROL'})
        # Create the sub SMACH state machine for Motion Control Phase 
        sm_motion = smach.StateMachine(['task_complete'])
        
        with sm_motion:
            smach.StateMachine.add('VISION_PROCESSING',
                         MotionControlProcess(),
                         transitions={'vision_processed':'VISION_ACTION','sub_task_complete':'task_complete'},
                         remapping={'locomotion_out':'locomotion',
                                    'movement_success_in':'success'}
                         )
            def vision_goal_cb(userdata, goal):
                vision_goal = PID_Controller.msg.ControllerGoal()
                #vision_goal
                rospy.loginfo('goal supplied!')
                return vision_goal
       
            def vision_result_cb(userdata, status, result):
                if status == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo('actionros completed!')
                return 'succeeded'
              
            smach.StateMachine.add('VISION_ACTION', SimpleActionState('LocomotionServer',
                                    PID_Controller.msg.ControllerAction,
                                goal_cb=vision_goal_cb,
                                result_cb=vision_result_cb,
                                outcomes=['succeeded','aborted','preempted'],
                                input_keys=['locomotion_in'],
                                output_keys=['movement_success_out']),
                             transitions={'succeeded':'VISION_PROCESSING','aborted':'VISION_PROCESSING','preempted':'VISION_PROCESSING'},
                             remapping={'locomotion_in':'locomotion_','movement_success_out':'success'}
                             )
        smach.StateMachine.add('MOTION_CONTROL', sm_motion,
                           transitions={'task_complete':'DISENGAGED'})
            
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('gate_server', sm_top, '/MISSION/GATE')
    sis.start()
   # gate = Gate()
    try:
        outcome = sm_top.execute()
        rospy.spin()
    except KeyboardInterrupt:
        sis.stop()
        print "Shutting down"
        
    