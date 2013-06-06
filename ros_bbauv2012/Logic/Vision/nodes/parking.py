#!/usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib
import sys
import collections
from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import ParkingCfg

#OpenCV imports
import cv2
from cv2 import cv as cv
from cv_bridge import CvBridge, CvBridgeError

#ROS Messages and Services
import PID_Controller.msg
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest
import numpy as np
from actionlib_msgs.msg._GoalStatus import GoalStatus

# State Machine
import smach
import smach_ros
from smach_ros import IntrospectionServer
from smach import StateMachine
from smach_ros import SimpleActionState

class Disengage(smach.State):
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
#             try:
#                 # Vision task querying mission server
#                 resp = mission_srv_request(False,True,locomotionGoal)
#             except rospy.ServiceException, e:
#                 print "Service call failed: %s"%e
        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Deactivating Park Node")
                return 'complete_outcome'
            if isStart:
                return 'start_complete'
        return 'aborted'

class Search(smach.State):
    global park
    global mission_srv_request
#    global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete','aborted'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing state SEARCH')
        while (not park.targetLockStatus) and not rospy.is_shutdown():
            #print len(gate.centroidx)
            rospy.sleep(0.05)
        if rospy.is_shutdown():
            return 'aborted'
        else:
#            try:
#                resp = mission_srv_request(True,False,None)
#            except rospy.ServiceException, e:
#                print "Service call failed: %s"%e
            return 'search_complete'

class MotionControlProcess(smach.State):
    global park
    isCorrection = False
    side_Kp = 0.008
    depth_Kp = 0.001
    forward = 0
    sidemove = 0    
    #Heading and Depth for vision task individual testing
    depth = 1.7
    heading = 115
    #Distance between center of image and center of target
    side_thresh = 25
    depth_thresh = 50
    #Area of contour to gauge how far vehicle is from target
    area_thresh = 1200
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['task_complete','aborted'],
                             output_keys=['complete_output'],
                             )
                    
    def execute(self,userdata):
    
        rospy.loginfo("Executing state MOTION CONTROL")
        actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        rospy.logdebug("Waiting For Action Server")
        actionClient.wait_for_server()
        rospy.logdebug("Action Server Ok")
        
        #Code below is to initialize vehicle position when individual testing vision task
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=self.heading,depth_setpoint=self.depth,sidemove_setpoint=self.sidemove)
        actionClient.send_goal(goal)
        rospy.logdebug("Initalizing") 
        actionClient.wait_for_result(rospy.Duration(0.5,0))
        rospy.logdebug("Done Initalizing") 
       
        while(not rospy.is_shutdown()):
               
#            rospy.loginfo("Target Still Far") 
            if park.area < self.area_thresh:
            
                #Adjust first if error is too large
                if  abs(park.errorSide) > self.side_thresh or abs(park.errorDepth) > self.depth_thresh:                    
                    goal.forward_setpoint = 0
                    goal.sidemove_setpoint = park.errorSide * self.side_Kp * -1
                    goal.depth_setpoint += park.errorDepth * self.depth_Kp * -1
                    goal.heading_setpoint = self.heading
                    actionClient.send_goal(goal)                    
                    adjusting = 'Adjusting. errSide= %d side=%d, errDepth=%d depth=%d area=%d' % (park.errorSide, self.sidemove, park.errorDepth, self.depth, park.area)
                    rospy.loginfo(adjusting)       
#                    actionClient.wait_for_result(rospy.Duration(5,0))                      
                    actionClient.wait_for_result()                      
                #Ok, now move forward
                if abs(park.errorSide) <= self.side_thresh and abs(park.errorDepth) <= self.depth_thresh:
                
                    goal.forward_setpoint = 1
                    goal.sidemove_setpoint = 0
                    goal.heading_setpoint = self.heading
                    actionClient.send_goal(goal)                 
                    forward = 'Forward! errSide= %d side=%d, errDepth=%d depth=%d area=%d' % (park.errorSide, self.sidemove, park.errorDepth, self.depth, park.area)
                    rospy.loginfo(forward)
#                    actionClient.wait_for_result(rospy.Duration(1,0))
                    actionClient.wait_for_result()                 
                         
            if park.area>=self.area_thresh: 
#                rospy.loginfo("Target Near Enough; Executing Last Maneuver")
                #Final State to change depth and move forward
                goal.forward_setpoint = 0
                goal.sidemove_setpoint = 0
                goal.heading_setpoint = self.heading
                goal.depth_setpoint -= 0.7
                actionClient.send_goal(goal)                                                
                final1 = 'Going for final: change depth area=%d' % (park.area) 
                rospy.loginfo(final1)
                actionClient.wait_for_result()
                                
                goal.forward_setpoint = 5
                goal.sidemove_setpoint = 0
                goal.heading_setpoint = self.heading
                actionClient.send_goal(goal)                                                
                final2 = 'Going for final2: go forward area=%d' % (park.area) 
                rospy.loginfo(final2)
                actionClient.wait_for_result()                              
#                actionClient.cancel_all_goals()    
                userdata.complete_input = True
            
#            if park.targetLockStatus==False:
#                pass
            
#                try:
#                   resp = mission_srv_request(False,True,locomotionGoal)
#                except rospy.ServiceException, e:
#                   print "Service call failed: %s"%e
                return 'task_complete'                

        return 'aborted'

'''
###################################################################

                       COMPUTER VISION CLASS 
        
###################################################################

'''

class Parking_Proc(smach.State):
    def __init__(self): 
    
        self.image_sub = rospy.Subscriber('stereo_camera/left/image_rect_color'
, Image, self.image_callback)
        self.image_pub = rospy.Publisher('/Vision/image_filter',Image)
                
        self.bridge = CvBridge()
        
        #CV parameters
        self.debug_mode = rospy.get_param("~debug_mode", True) 
        self.hmin = rospy.get_param("~hmin", 0)
        self.hmax = rospy.get_param("~hmax", 10)
        self.smin = rospy.get_param("~smin", 70)
        self.smax = rospy.get_param("~smax", 255)
        self.vmin = rospy.get_param("~vmin", 0)
        self.vmax = rospy.get_param("~vmax", 255)
        self.closeiter = rospy.get_param("~closeiter",5)
        self.openiter = rospy.get_param("~openiter",5)
        self.conArea = rospy.get_param("~conArea",120)
        self.conPeri = rospy.get_param("~conPeri",5)
        self.aspectRatio = rospy.get_param("~aspectRatio",7)
        
        #Detection Criteria parameters
        #http://stackoverflow.com/questions/5944708/python-forcing-a-list-to-a-fixed-size
        self.targetLockHistory = collections.deque(maxlen=100)
        self.targetXHistory = collections.deque(maxlen=20)
        self.targetYHistory = collections.deque(maxlen=20)
        self.targetLockHistoryThresh = rospy.get_param("~targetLockHistoryThresh", 50)
        self.XstdDevThresh = rospy.get_param("~XstdDevThresh", 50)
        self.YstdDevThresh = rospy.get_param("~YstdDevThresh", 50)
        

        #Globals used by motion control and vision state machine 
        self.targetLockStatus = False
        self.errorSide = 0
        self.errorDepth = 0
        self.area = 0        


        #dictionary used to make use of Jon's helper callback function
        self.params = {'hmin':self.hmin,'hmax':self.hmax,'smin':self.smin,'smax':self.smax,'vmin':self.vmin,'vmax':self.vmax }
        
        print "image processing"
            
    def image_callback(self, data):
        frame = self.convert_image(data)
        processed_image = self.process_image(frame)
        
    def convert_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv(ros_image, "bgr8")       
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

    def drcallback(config, level):
        rospy.logdebug(""" Reconfigure Req: {h_min},{h_max}""".format(**config))
        return config
            
    def process_image(self, frame):
    
        # Basic noise removal
        frame = cv2.GaussianBlur(frame, (3,3),0)   
        # outputs a binary image that falls within range of the HSV
        horizontal_green = self.hsv_thresholding(frame, hmin=self.params['hmin'], hmax=self.params['hmax'], smin=self.params['smin'], smax=self.params['smax'], vmin=self.params['vmin'], vmax=self.params['vmax']) 
        
        #perform closing operation to fill in the detected blobs from HSV Thresh
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(6,6),(3,3)) 
        horizontal_green = cv2.morphologyEx(horizontal_green, cv2.MORPH_OPEN, kernel, iterations=self.openiter)      
        horizontal_green = cv2.morphologyEx(horizontal_green, cv2.MORPH_CLOSE, kernel,iterations=self.closeiter)
        
        #making a copy because contour analysis alteres the image
        contourFrame = horizontal_green.copy()
        #finding and drawing contours
        target_contour = self.contour_analysis(contourFrame, frame, conArea=self.conArea, conPeri=self.conPeri, conAspRatio=self.aspectRatio)
        
        #Checking and publishing lock status of vision processing
        #target_contour[1] --> x coordinates
        #target_contour[2] --> y coordinates
        #targetLockHistory is a 1 if target is detected
        self.targetLockHistory.append(target_contour[3])
        self.targetXHistory.append(target_contour[1])
        self.targetYHistory.append(target_contour[2])
        
        
        if sum(self.targetLockHistory) >= self.targetLockHistoryThresh and sum(self.targetLockHistory) != 0:
            calcXStDev = np.array(self.targetXHistory)
            calcYStDev = np.array(self.targetYHistory)
            XstdDev = calcXStDev.std(axis=0)
            YstdDev = calcYStDev.std(axis=0)
#            print "Xsd = %d Ysd = %d " % (XstdDev, YstdDev)
            if XstdDev <= self.XstdDevThresh and YstdDev <= self.YstdDevThresh:
                self.targetLockStatus = True
                self.errorDepth = 240 - target_contour[2]
                self.errorSide = 320 - target_contour[1]
                error = 'Lock Status True: errDep = %d errSide = %d' % (self.errorDepth, self.errorSide) 
#                rospy.logdebug(error)
            else:
                self.targetLockStatus = False
        else:
            self.targetLockStatus = False
        
        #Draw target circle for visual debuggin
        cv2.circle(frame, (320,240), 12.5, (255,0,0), 2)
        self.image_pub.publish(self.bridge.cv_to_imgmsg(frame))

########################################################################

        # Trackbars and Windows for debugging purposes               
        if self.debug_mode:
            #callback functions for trackbar
            def paramSetter(key):
                def setter(val):    
	                self.params[key] = val
                return setter            
            cv2.namedWindow("HSV Settings", cv2.WINDOW_NORMAL
)
            cv2.createTrackbar("H min:", "HSV Settings", self.params['hmin'], 180, paramSetter('hmin'));
            cv2.createTrackbar("H max:", "HSV Settings", self.params['hmax'], 180, paramSetter('hmax'));
            cv2.createTrackbar("S min:", "HSV Settings", self.params['smin'], 255, paramSetter('smin'));
            cv2.createTrackbar("S max:", "HSV Settings", self.params['smax'], 255, paramSetter('smax'));
            cv2.createTrackbar("V min:", "HSV Settings", self.params['vmin'], 255, paramSetter('vmin'));                        
            cv2.createTrackbar("V max:", "HSV Settings", self.params['vmax'], 255, paramSetter('vmax'));
            cv2.imshow("HSV Parking", horizontal_green)
            cv2.imshow('Contours', frame)
                          
            #attempting to arrange windows; opCV unable to make it more automatic
            l = 325
            w = 245
            cv2.moveWindow("HSV Settings",0,0)
            cv2.moveWindow("HSV Parking",l,0)
            cv2.moveWindow("Contours",l*2,0)            

                       
        cv.WaitKey(5)
            
        return frame

########################################################################

    def contour_analysis(self, contour_frame, input_frame, conArea=120, conPeri=50, conAspRatio=7):
    
        contours , hierarchy = cv2.findContours(contour_frame, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #selecting the right contour based on area, perimeter and aspect ratio
        #asepct ratio seems good to get horizontal rectangular contours
        output = []    
        cnt = None
        
        
        def approxAndDrawContour(input_frame, input_contour):
            #appoximating contour to get better rectangle
            approx = cv2.approxPolyDP(input_contour,0.1*cv2.arcLength(input_contour,True),True)        
            x,y,w,h = cv2.boundingRect(approx)
            target_x = int(x+(w/2)) 
            target_y = int(y+(h/2))
            
            #Coordinates used for visual motion
            targetCoord = 'x = %d y = %d' % (target_x, target_y) 
            #rospy.loginfo(targetCoord)

            #draw contour on input_frame    
            cv2.drawContours(input_frame, [approx], 0, (0,0,255), 2)
            #drawing tracking point
            cv2.circle(input_frame, (target_x,target_y), 5, (0,255,255),-1)
            output.append(cnt)
            output.append(target_x)
            output.append(target_y)
            output.append(1)
            return output
                              
        
        for contour in contours:
            area = cv2.contourArea(contour)
            # updating area for Motion contorl to control forward
            perimeter = cv2.arcLength(contour, True)
            x,y,w,h = cv2.boundingRect(contour)
            aspect_ratio = int(w)/h
        
#            test for the correct contour in list of contours
            if area >= conArea and perimeter >= conPeri and aspect_ratio >= conAspRatio:
                contourDescrip = 'area=%d peri=%d aspectRatio=%d' % (area, perimeter, aspect_ratio)
                #rospy.logdebug(contourDescrip)
                
                cnt = contour
                self.last_cnt = cnt
                contourAnalysisOutput = approxAndDrawContour(input_frame, cnt)
                self.area = area
                break
            
        if cnt == None:
            #rospy.logdebug("No Contours Found")
            output = []
            for i in range(4):
                output.append(0)
                    
        return output
		
    def hsv_thresholding(self, input_frame, hmin=0, hmax=180, smin=0, smax=255, vmin=0, vmax=255):
        
        #Assume frame is BGR format
        hsv_img = cv2.cvtColor(input_frame,cv2.COLOR_BGR2HSV)
        hsv_channels = cv2.split(hsv_img)

        hue = cv2.equalizeHist(hsv_channels[0])
        sat = cv2.equalizeHist(hsv_channels[1])
        val = cv2.equalizeHist(hsv_channels[2])

        hsv_equalized = cv2.merge([hue, sat, val])
#        cv2.imshow('hsv_equalized', hsv_equalized)
                
        #initialize matrix for HSV min and max values
        COLOR_MIN = np.array([hmin, smin, vmin],np.uint8)
        COLOR_MAX = np.array([hmax, smax, vmax],np.uint8)
        
        #Execute Thresholding
        hsv_threshed = cv2.inRange(hsv_equalized, COLOR_MIN, COLOR_MAX)
        
        return hsv_threshed

def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    rospy.loginfo("Parking service handled.")
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
         
parking = None
isAbort = False
isStart = True
isEnd = False
locomotionGoal = controller()
mission_srv_request = None
vision_srv = None
movement_client = None

# To test just the vision code, comment everything, uncomment the last 2 lines
# To test without mission, uncomment "Service Client" and all mission_srv_request calls

if __name__ == '__main__':
    rospy.init_node('Park', log_level=rospy.INFO, anonymous=False)
    drserver = Server(ParkingCfg, drcallback)
    
    vision_srv = rospy.Service('park_srv', mission_to_vision, handle_srv)
    rospy.loginfo('park_srv initialized!')
    
    #Service Client. This part is commented if testing Vision task state machine without mission planner
#    rospy.loginfo('Park waiting for mission_srv...')
#    rospy.wait_for_service('mission_srv')
#    mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission)
#    rospy.loginfo('Park connected to mission_srv!')
    
    #Computer vision processing instantiation
    park = Parking_Proc()
    
    sm_top = smach.StateMachine(outcomes=['park_complete','aborted'])
    #Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED', Disengage(), transitions={'start_complete':'SEARCH','complete_outcome':'park_complete','aborted':'aborted'}, remapping={'complete_input':'goal'})
        smach.StateMachine.add('SEARCH', Search(), transitions={'search_complete':'MOTION_CONTROL','aborted':'aborted'})    
        smach.StateMachine.add('MOTION_CONTROL', MotionControlProcess(), transitions={'task_complete': 'DISENGAGED','aborted':'aborted'}, remapping={'complete_output':'goal'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('park_server', sm_top, '/MISSION/PARK')
    sis.start()
    sm_top.userdata.goal = False

    try:
        outcome = sm_top.execute()
        rospy.spin()
    except KeyboardInterrupt:
        sis.stop()
        print "Shutting down"
#    Parking_Proc()
#    rospy.spin()
    

