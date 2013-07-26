#!/usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib
import sys
import collections
from dynamic_reconfigure.server import Server
from Vision.cfg import ParkingConfig

#OpenCV imports
import cv2
from cv2 import cv as cv
from cv_bridge import CvBridge, CvBridgeError
from com.histogram.histogram import bbHistogram
from com.histogram.histogram import Hist_constants

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

'''
###################################################################

                       COMPUTER VISION CLASS 
        
###################################################################

'''

class Parking_Proc():
    
    def __init__(self): 
    
        self.image_sub = None
        self.image_pub = None
        self.green_hist = None
        
        self.bridge = CvBridge()
        #http://stackoverflow.com/questions/5944708/python-forcing-a-list-to-a-fixed-size
        self.targetLockHistory = collections.deque(maxlen=100)
        self.targetXHistory = collections.deque(maxlen=20)
        self.targetYHistory = collections.deque(maxlen=20)
        
        #Globals used by motion control and vision state machine 
        self.targetLockStatus = False
        self.errorSide = 0
        self.errorDepth = 0
        self.area = 0        
        self.image_pub = rospy.Publisher('/Vision/image_filter', Image)
        
    def register(self):
        global test_mode

#        if not test_mode:
        self.image_sub = rospy.Subscriber('/stereo_camera/left/image_rect_color', Image, self.image_callback)
#        if test_mode:
#        self.image_sub = rospy.Subscriber('/stereo_camera/left/image_rect_color_opt_tc', Image, self.image_callback)

        rospy.logdebug('Registered')
        
    def unregister(self):
        self.image_sub.unregister()
        rospy.logdebug('Unregistered')
    
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
                    
    def process_image(self, frame):

        hist_frame = frame    
        # Basic noise removal
        frame = cv2.GaussianBlur(frame, (3,3),0)   
        # outputs a binary image that falls within range of the HSV
        horizontal_green = self.hsv_thresholding(frame, hmin=params['hueLow'], hmax=params['hueHigh'], smin=params['satLow'], smax=params['satHigh'], vmin=params['valLow'], vmax=params['valHigh']) 
        
        #perform closing operation to fill in the detected blobs from HSV Thresh
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(6,6),(3,3)) 
        horizontal_green = cv2.morphologyEx(horizontal_green, cv2.MORPH_OPEN, kernel, iterations=params['openiter'])      
        horizontal_green = cv2.morphologyEx(horizontal_green, cv2.MORPH_CLOSE, kernel,iterations=params['closeiter'])
        
        #making a copy because contour analysis alteres the image
        contourFrame = horizontal_green
        #finding and drawing contours
        target_contour = self.contour_analysis(contourFrame, frame, conArea=params['conArea'], conPeri=params['conPeri'], conAspRatio=params['aspectRatio'])
        
        #Checking and publishing lock status of vision processing
        #target_contour[1] --> x coordinates
        #target_contour[2] --> y coordinates
        #targetLockHistory is a 1 if target is detected
        self.targetLockHistory.append(target_contour[3])
        self.targetXHistory.append(target_contour[1])
        self.targetYHistory.append(target_contour[2])
        
        if sum(self.targetLockHistory) >= params['targetLockHistoryThresh'] and sum(self.targetLockHistory) != 0:
            calcXStDev = np.array(self.targetXHistory)
            calcYStDev = np.array(self.targetYHistory)
            XstdDev = calcXStDev.std(axis=0)
            YstdDev = calcYStDev.std(axis=0)
#            print "Xsd = %d Ysd = %d " % (XstdDev, YstdDev)
            if XstdDev <= params['XstdDevThresh'] and YstdDev <= params['YstdDevThresh']:
                self.targetLockStatus = True
                self.errorDepth = 240 - target_contour[2]
                self.errorSide = 320 - target_contour[1]
                error = 'Lock Status True: errDep = %d errSide = %d' % (self.errorDepth, self.errorSide) 
#                rospy.logdebug(error)
            else:
                self.targetLockStatus = False
        else:
            self.targetLockStatus = False   

########################################################################

        # Trackbars and Windows for debugging purposes               
        if params['debug_mode'] ==1:
#            cv2.imshow("HSV Parking", horizontal_green)
            self.green_hist = bbHistogram("green",Hist_constants.TRIPLE_CHANNEL)
            self.green_hist.setParams(params)
            self.green_hist.getTripleHist(hist_frame)
#            cv2.imshow('Contours', frame)                          
            #attempting to arrange windows; opCV unable to make it more automatic
#            l = 325
#            w = 245
#            cv2.moveWindow("HSV Parking",l,0)
#            cv2.moveWindow("Contours",l*2,0)     

        if params['debug_mode'] ==0:
            self.green_hist = None
              
        cv.WaitKey(1)

#        debug_frame = cv2.cv.fromarray(horizontal_green)
#        self.image_pub.publish(self.bridge.cv_to_imgmsg(debug_frame)) #, encoding="bgr8"

        return frame

########################################################################

    def contour_analysis(self, contour_frame, input_frame, conArea=120, conPeri=50, conAspRatio=7):
    
        contour_frame_2 = contour_frame.copy() #Change this shit out later

        contours , hierarchy = cv2.findContours(contour_frame_2, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

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

#            #draw contour on input_frame    
#            cv2.drawContours(input_frame, [approx], 0, (0,0,255), 2)
#            #drawing tracking point
#            cv2.circle(input_frame, (target_x,target_y), 5, (0,255,255),-1)
            
            #Prepare Debug Frame for publishing
            debug_frame = np.zeros((480,640,3), dtype=np.uint8) #blank image
            trip_input_frame = cv2.merge([input_frame, input_frame, input_frame])
            debug_frame += trip_input_frame
            cv2.drawContours(debug_frame, [approx], 0, (0,0,255), 2)
#            #drawing tracking point
            cv2.circle(debug_frame, (target_x,target_y), 5, (0,255,255),-1)
#            #Draw target circle for visual debuggin
            cv2.circle(debug_frame, (320,240), 12, (255,0,0), 2)
            debug_frame= cv2.cv.fromarray(debug_frame)
            self.image_pub.publish(self.bridge.cv_to_imgmsg(debug_frame, encoding="bgr8"))

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
                rospy.logdebug(contourDescrip)
                
                cnt = contour
                self.last_cnt = cnt
                contourAnalysisOutput = approxAndDrawContour(contour_frame, cnt)
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

        hue = hsv_channels[0] #cv2.equalizeHist(hsv_channels[0])
        sat = hsv_channels[1] #cv2.equalizeHist(hsv_channels[1])
        val = hsv_channels[2] #cv2.equalizeHist(hsv_channels[2])

        hsv_equalized = cv2.merge([hue, sat, val])
#        cv2.imshow('hsv_equalized', hsv_equalized)
                
        #initialize matrix for HSV min and max values
        COLOR_MIN = np.array([hmin, smin, vmin],np.uint8)
        COLOR_MAX = np.array([hmax, smax, vmax],np.uint8)
        
        #Execute Thresholding
        hsv_threshed = cv2.inRange(hsv_equalized, COLOR_MIN, COLOR_MAX) #equalization removed

        return hsv_threshed

'''
###################################################################

               SMACH STATE MACHINE CLASS DECLARATION
        
###################################################################

'''     

class Disengage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete', 'completed'],
                            input_keys=['complete_input'])
    def execute(self,userdata):
        global locomotionGoal
        global isStart
        global isAbort
        global isEnd        
        global park

        isStart = False
        isAbort = False

        r = rospy.Rate(20)
        while (not rospy.is_shutdown()):
            if isEnd:
                return 'completed'
            if isStart:
                park.register()
                rospy.sleep(3)
                return 'start_complete'
            r.sleep()

class Search(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete','aborted'])
        
    def execute(self,userdata):
        global park
        global isAbort
        global mission_srv
        
        rospy.loginfo('Executing state SEARCH')
        
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            #search conditions here
            if park.targetLockStatus:
                try:
                    resp = mission_srv(True,False,None)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                return 'search_complete' 
            
            if isAbort:
                park.unregister()
                return 'aborted'               
            r.sleep()

class MotionControlProcess(smach.State):
    
    #params['side_thresh'] & params['depth_thresh']: Allowable distance between center of image and center of target
    #params['area_thresh']: Area of contour to gauge how far vehicle is from target; after which vehicle goes into final
    #params['approach_area_thresh']: When vehicle is far allow vehicle to take longer to adjust side & depth
        
    def __init__(self):
        smach.State.__init__(self, outcomes=['task_complete','aborted'],
                             output_keys=['complete_output'],
                             )
                    
    def execute(self,userdata):
    
        global park
        global locomotionGoal
        global isEnd
        global set_LocoMode
        
        rospy.loginfo("Executing state MOTION CONTROL")
        actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        rospy.logdebug("Waiting For Action Server")
        actionClient.wait_for_server()
        rospy.logdebug("Action Server Ok")
        
        #Code below is to initialize vehicle position when individual testing vision task
        goal = bbauv_msgs.msg.ControllerGoal()
        goal.forward_setpoint = 0
        goal.sidemove_setpoint = 0
        goal.heading_setpoint = locomotionGoal.heading_setpoint 
        goal.depth_setpoint = locomotionGoal.depth_setpoint 
        actionClient.send_goal(goal)
        rospy.logdebug("Initalizing") 
        actionClient.wait_for_result(rospy.Duration(0.1,0))
        rospy.logdebug("Done Initalizing") 
        
        r = rospy.Rate(40)
        while(not rospy.is_shutdown()):
            
            if not park.targetLockStatus:
                actionClient.cancel_all_goals()

            if park.area < params['area_thresh']:
                if park.area < params['approach_area_thresh']:
                    wait_time = params['approachWaitTime']
                if park.area > params['approach_area_thresh']:
                    wait_time = params['finalWaitTime']
                    
                #Adjust first if error is too large
                if  abs(park.errorSide) > params['side_thresh'] or abs(park.errorDepth) > params['depth_thresh']:     

                    #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
                    try:
                        resp = set_LocoMode(False, True)
                        rospy.loginfo("LocoMode set to Sidemove")
                    except rospy.ServiceException, e:
                        rospy.loginfo("LocoMode Fwd NOT set: %s" % e)
          
                    goal.forward_setpoint = 0
                    goal.sidemove_setpoint = park.errorSide * params['side_Kp'] * -1
                    goal.depth_setpoint += park.errorDepth * params['depth_Kp'] * -1
                    
                    if goal.depth_setpoint <= 0.5:
                        goal.depth_setpoint = 0.5
                        rospy.loginfo("Parking tried to bring vehicle to surface; setting depth to 0.5")

                    actionClient.send_goal(goal)                    
                    rospy.loginfo('Approach: ADJUST. errSide= %d, errDepth=%d area=%d' % (park.errorSide, park.errorDepth, park.area))       
                    #actionClient.wait_for_result(rospy.Duration(5,0))                      
                    actionClient.wait_for_result(rospy.Duration(wait_time,0))                      
                #Ok, now move forward
                if abs(park.errorSide) <= params['side_thresh'] and abs(park.errorDepth) <= params['depth_thresh']:

                    #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
                    try:
                        resp = set_LocoMode(False, False)
                        rospy.loginfo("LocoMode set to Default")
                    except rospy.ServiceException, e:
                        rospy.loginfo("LocoMode Fwd NOT set: %s" % e)
                
                    goal.forward_setpoint = params['approachFwdDist']
                    goal.sidemove_setpoint = 0
                    actionClient.send_goal(goal)                 
                    rospy.loginfo('Approach: FOWARD! errSide= %d, errDepth=%d area=%d' % (park.errorSide, park.errorDepth, park.area))
#                    actionClient.wait_for_result(rospy.Duration(1,0))
                    actionClient.wait_for_result(rospy.Duration(wait_time,0))                 
                         
            if park.area>=params['area_thresh']: 
#                 rospy.loginfo("Target Near Enough; Executing Last Maneuver")
                #Final State to change depth and move forward

                #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
                try:
                    resp = set_LocoMode(False, False)
                    rospy.loginfo("LocoMode set to Default")
                except rospy.ServiceException, e:
                    rospy.loginfo("LocoMode Fwd NOT set: %s" % e)

                goal.forward_setpoint = 0
                goal.sidemove_setpoint = 0
                goal.heading_setpoint = ((goal.heading_setpoint-90)%360+360)%360
                if goal.depth_setpoint <= 0.5:
                    goal.depth_setpoint = 0.5                
                actionClient.send_goal(goal)                                                
                rospy.loginfo('Final: Heading change! area=%d' % (park.area))
                actionClient.wait_for_result(rospy.Duration(10,0))

                goal.forward_setpoint = 0.2
                goal.sidemove_setpoint = 0
                goal.depth_setpoint -= params['final_depthchange']
                if goal.depth_setpoint < 0.5:
                    goal.depth_setpoint = 0.5                
                actionClient.send_goal(goal)                                                
                rospy.loginfo('Final: Depth change! area=%d' % (park.area))
                actionClient.wait_for_result(rospy.Duration(10,0))

                #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
                try:
                    resp = set_LocoMode(False, True)
                    rospy.loginfo("LocoMode set to Sidemove")
                except rospy.ServiceException, e:
                    rospy.loginfo("LocoMode Fwd NOT set: %s" % e)
                                
                goal.forward_setpoint = 0
                goal.sidemove_setpoint = params['final_moonwalk']
                actionClient.send_goal(goal)                                                
                rospy.loginfo('Final: Moonwalking!')
                actionClient.wait_for_result(rospy.Duration(40,0))

                #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
                try:
                    resp = set_LocoMode(False, False)
                    rospy.loginfo("LocoMode set to Default")
                except rospy.ServiceException, e:
                    rospy.loginfo("LocoMode Fwd NOT set: %s" % e)

                goal.forward_setpoint = 0
                goal.sidemove_setpoint = 0
                goal.heading_setpoint = ((goal.heading_setpoint+90)%360+360)%360
                actionClient.send_goal(goal)                                                
                rospy.loginfo('Final: Turning Back')
                actionClient.wait_for_result(rospy.Duration(10,0))
                actionClient.cancel_all_goals()  
                
                try:
                   locomotionGoal.heading_setpoint = goal.heading_setpoint
                   locomotionGoal.depth_setpoint = goal.depth_setpoint
                   resp = mission_srv(False,True,locomotionGoal)
                except rospy.ServiceException, e:
                   print "Service call failed: %s"%e
                
                isEnd = False   
                park.unregister()
                return 'task_complete'                

            if isAbort:
                park.unregister()
                return 'aborted'
            r.sleep()

def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    rospy.logdebug("service handled.")
    if req.start_request:
        rospy.logdebug("isStart true.")
        isStart = True
        # Format for service: start_response, abort_response
        locomotionGoal = req.start_ctrl
    if req.abort_request:
        rospy.logdebug("isAbort true.")        
        isAbort = True
    return mission_to_visionResponse(isStart,isAbort)

'''
###################################################################

                       MAIN PYTHON THREAD
        
###################################################################

'''
         
#Globals
#vision processing object
parking = None
#whether to abort
isAbort = False
#whether to start search
isStart = True  
isEnd = False
#used to obtain seed goal and return to mission the final goal; depth and heading are absolute
locomotionGoal = controller()
#vision uses this to comms with mission
mission_srv = None
#vision server object; this is how mission comms with vision
vision_srv = None

set_LocoMode = None

params = {'hueLow':0, 'hueHigh':0, 'satLow':0, 'satHigh':0,'valLow':0, 'valHigh':0, 
          'closeiter':0, 'openiter':0, 'conArea':0, 'conPeri':0, 'aspectRatio':0, 
          'targetLockHistoryThresh':0, 'XstdDevThresh':0, 'YstdDevThresh':0, 'debug_mode':0, 
          'side_thresh': 0, 'depth_thresh': 0, 'area_thresh': 0, 'approach_area_thresh': 0, 'side_Kp': 0, 'depth_Kp':0, 
          'approachFwdDist':0, 'approachWaitTime':0, 'finalWaitTime':0, 'final_depthchange': 0, 'final_moonwalk': 0}

# To test just the vision code, comment everything, uncomment the last 2 lines
# To test without mission, uncomment "Service Client" and all mission_srv calls

if __name__ == '__main__':
    rospy.init_node('Park', log_level=rospy.INFO, anonymous=False)
    
    test_mode = rospy.get_param('~testmode',False)
    
    def configCallback(config, level):
        for param in params:
            params[param] = config[param]
        return config
    drserver = Server(ParkingConfig, configCallback)
    
    vision_srv = rospy.Service('park_srv', mission_to_vision, handle_srv)
    rospy.loginfo('park_srv initialized!')
    
    if not test_mode:      
        #Service Client. This part is commented if testing Vision task state machine without mission planner
        rospy.loginfo('Park waiting for mission_srv...')
        rospy.wait_for_service('mission_srv')
        mission_srv = rospy.ServiceProxy('mission_srv', vision_to_mission, headers={'id':2})
        rospy.loginfo('Park connected to mission_srv!')
        
        #Getting ready service to change locomotion mode
        rospy.loginfo('Parking Waiting for Locomotion Modes Service to start up...')
        rospy.wait_for_service('locomotion_mode_srv')
        set_LocoMode = rospy.ServiceProxy('locomotion_mode_srv',locomotion_mode)
        rospy.loginfo('Parking Connected to Locomotion Mode Service')
          
        #Computer vision processing instantiation
        park = Parking_Proc()
          
        sm_top = smach.StateMachine(outcomes=['park_complete','park_failed'])
        #Add overall States to State Machine for Gate Task 
        with sm_top:
            smach.StateMachine.add('DISENGAGED', Disengage(), transitions={'start_complete':'SEARCH', 'completed':'park_complete'})
            smach.StateMachine.add('SEARCH', Search(), transitions={'search_complete':'MOTION_CONTROL','aborted':'DISENGAGED'})    
            smach.StateMachine.add('MOTION_CONTROL', MotionControlProcess(), transitions={'task_complete': 'DISENGAGED','aborted':'DISENGAGED'})
      
        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('park_server', sm_top, '/MISSION/PARK')
        sis.start()
      
        try:
            outcome = sm_top.execute()
    #        rospy.spin()
        except KeyboardInterrupt:
            sis.stop()
            print "Shutting down"

        rospy.signal_shutdown("Deactivating Park Node")

    if test_mode:
        park = Parking_Proc()
        park.register()
        rospy.spin()
    

