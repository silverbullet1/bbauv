#!/usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy

import cv2
from cv2 import cv as cv
from cv_bridge import CvBridge, CvBridgeError
from bbauv_msgs.msg import visionTaskOutput

from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest
import numpy as np
import sys
import collections

# State Machine
import smach
import smach_ros
from smach_ros import IntrospectionServer

class Countdown(smach.State):
    def __init__(self, sleep=1.0):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        self.sleep_time = sleep
    
    def execute(self, userdata):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
            if self.preempt_requested():
                self.services_preempt()
                return 'preempted'
        return 'succeeded'

class Align(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'stopped'], input_keys=['targetLockStatus_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Align')

        if userdata.targetLockStatus_in == True:
            print 'Aligning'
            return 'succeeded'

        if userdata.targetLockStatus_in == False:            
            print "Cannot align without lock to target" 
            return 'stopped'
            
        if userdata == None:
            return 'stopped'


class SlowAhead():
    def __init__(self):
        smach.State.__init__(self, outcomes=[])

    def execute(self, userdata):
        print "Slow Ahead"    

class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Begin'])
        self.start_signal = rospy.get_param("~start_signal", False)        
    def execute(self, userdata):
        print "Standby"
        while self.start_signal == False: 
            self.start_signal = rospy.get_param("~start_signal", False) 
            if self.start_signal == True:
                return 'Begin'
                break

class Final():
    def __init__(self):
        smach.State.__init__(self, outcomes=[])

    def execute(self, userdata):
        print "On Final"    
        
class Initialization:
    def __init__(self):
        self.node_name = "Parking"
        rospy.init_node(self.node_name)
        rospy.loginfo("Starting Node " + str(self.node_name))
        print "init'ing'"

class Parking_Proc(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['Lock', 'No Lock'])
        
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback)
        self.visionTaskOutputPub = rospy.Publisher('/ParkingVisionTaskOutpout',  visionTaskOutput)
        self.visionTaskOutputMsg = visionTaskOutput()
        
        self.bridge = CvBridge()
        
        self.debug_mode = rospy.get_param("~debug_mode", True) 
        
        #http://stackoverflow.com/questions/5944708/python-forcing-a-list-to-a-fixed-size
        self.targetLockHistory = collections.deque(maxlen=100)
        self.targetXHistory = collections.deque(maxlen=20)
        self.targetYHistory = collections.deque(maxlen=20)
        self.targetLockStatus = False
        self.targetLockHistoryThresh = rospy.get_param("~targetLockHistoryThresh", 50)
        self.XstdDevThresh = rospy.get_param("~XstdDevThresh", 50)
        self.YstdDevThresh = rospy.get_param("~YstdDevThresh", 50)
        
        self.debug_mode = rospy.get_param("~debug_mode", True)       
        
        self.hmin = rospy.get_param("~hmin", 0)
        self.hmax = rospy.get_param("~hmax", 10)
        self.smin = rospy.get_param("~smin", 70)
        self.smax = rospy.get_param("~smax", 255)
        self.vmin = rospy.get_param("~vmin", 0)
        self.vmax = rospy.get_param("~vmax", 255)
        
        self.closeiter = rospy.get_param("~closeiter",5)
        self.openiter = rospy.get_param("~openiter",5)

        #dictionary used to make use of Jon's helper callback function
        self.params = {'hmin':self.hmin,'hmax':self.hmax,'smin':self.smin,'smax':self.smax,'vmin':self.vmin,'vmax':self.vmax }
        
        print "image processing"
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Parking_Proc')
        
        if self.targetLockStatus == True:
            return 'Lock'
        else:
            return 'No Lock'
            
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
        target_contour = self.contour_analysis(contourFrame, frame, conArea=120, conPeri=50, conAspRatio=7)
        
        #Checking and publishing lock status of vision processing
        self.targetLockHistory.append(target_contour[3])
        self.targetXHistory.append(target_contour[1])
        self.targetYHistory.append(target_contour[2])
        self.visionTaskOutputMsg.xCoordinates = target_contour[1]
        self.visionTaskOutputMsg.yCoordinates = target_contour[2]
        
        if sum(self.targetLockHistory) >= self.targetLockHistoryThresh and sum(self.targetLockHistory) != 0:
            calcXStDev = np.array(self.targetXHistory)
            calcYStDev = np.array(self.targetYHistory)
            XstdDev = calcXStDev.std(axis=0)
            YstdDev = calcYStDev.std(axis=0)
#            print "Xsd = %d Ysd = %d " % (XstdDev, YstdDev)
            if XstdDev <= self.XstdDevThresh and YstdDev <= self.YstdDevThresh:
                self.targetLockStatus = True
                self.visionTaskOutputMsg.lockStatus = True
            else:
                self.targetLockStatus = False
        else:
            self.targetLockStatus = False
        
        self.visionTaskOutputMsg.lockStatus = self.targetLockStatus
        self.visionTaskOutputPub.publish(visionTaskOutputMsg)
            
#        print self.targetLockStatus
        
        #Draw target circle for visual debuggin
        cv2.circle(frame, (160,120), 7, (0,255,0), 1)
        cv2.imshow('Contours', frame)

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
#            cv2.createTrackbar("V min:", "HSV Settings", self.params['vmin'], 255, paramSetter('vmin'));                        
#            cv2.createTrackbar("V max:", "HSV Settings", self.params['vmax'], 255, paramSetter('vmax'));
            cv2.imshow("HSV Parking", horizontal_green)
            cv2.imshow('Contours', frame)
                          
            #attempting to arrange windows; opCV unable to make it more automatic
            l = 325
            w = 245
            cv2.moveWindow("HSV Settings",0,0)
            cv2.moveWindow("HSV Parking",l,0)
            cv2.moveWindow("Contours",l*2,0) 
                       
        cv.WaitKey(10)
            
        return frame

########################################################################

    def contour_analysis(self, contour_frame, input_frame, conArea=100, conPeri=100, conAspRatio=5):
    
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
    #                coordinates used for visual motion

            targetCoord = 'x = %d y = %d' % (target_x, target_y) 
            rospy.logdebug(targetCoord)

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
            perimeter = cv2.arcLength(contour, True)
            x,y,w,h = cv2.boundingRect(contour)
            aspect_ratio = int(w)/h
        
#            test for the correct contour in list of contours
            if area >= conArea and perimeter >= conPeri and aspect_ratio >= conAspRatio:
                contourDescrip = 'area=%d peri=%d aspectRatio=%d' % (area, perimeter, aspect_ratio)
                rospy.logdebug(contourDescrip)
                
                cnt = contour
                self.last_cnt = cnt
                contourAnalysisOutput = approxAndDrawContour(input_frame, cnt)
                break
            
        if cnt == None:
            rospy.logdebug("No Contours Found")
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
         
def main(args):

    try:
        Initialization()
        
        ParkingTaskStateMachine = smach.StateMachine(outcomes=['outcome1', 'preempted'])
        
        with ParkingTaskStateMachine:       
            smach.StateMachine.add('COUNTDOWN_START',Countdown(1.0),transitions={'succeeded':'Standby'})
            smach.StateMachine.add('Standby', Standby(), transitions={'Begin':'CON'})

#            smach.StateMachine.add('Parking_Proc', Parking_Proc(), transitions={'Lock':'CON', 'No Lock':'Parking_Proc'})
            
            sm_con = smach.Concurrence(outcomes=['outcome2','outcome3'], default_outcome='outcome3', outcome_map={'outcome2':{'Align':'succeeded','Parking_Proc_AfterLock':'Lock'}})
            
            with sm_con:
                smach.Concurrence.add('Parking_Proc_AfterLock', Parking_Proc())
                smach.Concurrence.add('Align', Align())

            smach.StateMachine.add('CON', sm_con, transitions={'outcome2':'CON', 'outcome3':'Standby'})


        

        sis = IntrospectionServer('ParkingTaskStateMachine_introsp', ParkingTaskStateMachine,'/SM_ROOT')
        sis.start()     
           
        outcome = ParkingTaskStateMachine.execute()
            
        rospy.spin()
        sis.stop()
    
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()   

if __name__ == '__main__':
    main(sys.argv)

