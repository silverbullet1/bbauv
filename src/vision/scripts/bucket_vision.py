#!/usr/bin/env python

import roslib; roslib.load_manifest('vision')

import rospy
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from dynamic_reconfigure.server import Server as DynServer

from bbauv_msgs.msg import * 
from bbauv_msgs.srv import *
import vision.cfg.bucketConfig as Config
from sensor_msgs.msg import Image

import signal
from collections import deque

import numpy as np

class BucketDetector:
    #HSV thresholds for red color
    lowThresh1 = np.array([ 92, 0, 10 ])
    hiThresh1 = np.array([ 132, 255, 255 ]) 
    areaThresh = 10000
    
    bridge = None
    
    curHeading = 0
    depth_setpoint = 0.3
    maniData = 0
    actionsHist = deque()
    
    screen = { 'width' : 640, 'height' : 480 }
    
    locomotionClient = actionlib.SimpleActionClient("LocomotionServer", ControllerAction)
        
    #Utility Methods  
    def rosimg2cv(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, ros_image.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)

        return frame 

    def __init__(self):
        self.testing = rospy.get_param("~testing", False)
        self.isAborted = False
        self.isKilled = False
        signal.signal(signal.SIGINT, self.userQuit)

        self.rectData = { 'detected' : False }
        self.bridge = CvBridge()

        #Initialize Subscribers and Publishers
        self.image_topic = rospy.get_param('~image', '/bot_camera/camera/image_raw')
        self.image_pub = rospy.Publisher("/Vision/image_filter", Image)
        self.register()
        
        # Setup dynamic reconfigure server
        self.dyn_reconf_server = DynServer(Config, self.reconfigure)

        #Initialize mission planner communication server and client
        self.comServer = rospy.Service("/bucket/mission_to_vision", mission_to_vision, self.handleSrv)
        if not self.testing: 
            self.toMission = rospy.ServiceProxy("/bucket/vision_to_mission", vision_to_mission)
            self.toMission.wait_for_service(timeout = 10)
        
        #Initializing controller service
        controllerServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        controllerServer(forward=True, sidemove=True, heading=True, depth=True, pitch=True, roll=True,
                         topside=False, navigation=False)

        #Make sure locomotion server is up
        try:
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.logerr("Locomotion server timeout!")
            self.isKilled = True

        #TODO: Add histogram modes for debug
        rospy.loginfo("Bucket ready")
            
    def userQuit(self, signal, frame):
        self.isAborted = True
        self.isKilled = True
        
    def reconfigure(self, config, level):
        rospy.loginfo("Got reconfigure request!")
        self.lowThresh1[0] = config['loH']
        self.lowThresh1[1] = config['loS']
        self.lowThresh1[2] = config['loV']
        
        self.hiThresh1[0] = config['hiH']
        self.hiThresh1[1] = config['hiS']
        self.hiThresh1[2] = config['hiV']
        
        self.areaThresh = config['area_thresh']
        
        return config

    def sendMovement(self, f=0.0, h=None, sm=0.0, d=None, recordAction=True):
        d = d if d else self.depth_setpoint
        h = h if h else self.curHeading
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=f, heading_setpoint=h,
                                             sidemove_setpoint=sm, depth_setpoint=d)
 
        # Record actions to revert if necessary
        if recordAction:
            if len(self.actionsHist) > 10:
                self.actionsHist.popleft()
            self.actionsHist.append([f, h, sm, d])
 
        rospy.loginfo("Moving f:{}, h:{}, sm:{}, d:{}".format(f, h, sm, d))
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(1.0))

    def revertMovement(self):
        if len(self.actionsHist) == 0:
            return False
        
        rospy.loginfo("Reverting...")
        movements = self.actionsHist.popleft()
        # Reverse the direction of forward and sidemove
        movements[0] = -movements[0]
        movements[2] = -movements[2]
        self.sendMovement(*movements, recordAction=False)
        return True

    def stopRobot(self):
        self.sendMovement(f=0.0, sm=0.0)

    def register(self):
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cameraCallback)
        self.headingSub = rospy.Subscriber('/euler', compass_data, self.compassCallback)
        self.maniSub = rospy.Subscriber('/manipulators', manipulator, self.maniCallback)
        rospy.loginfo("Topics registered")
        
    def unregister(self):
        self.image_sub.unregister()
        self.headingSub.unregister()
        rospy.loginfo("Topics unregistered")
    
    def handleSrv(self, req):
        if req.start_request:
            self.isAborted = False
            self.depth_setpoint = req.start_ctrl.depth_setpoint
        elif req.abort_request:
            self.isAborted = True
        return mission_to_visionResponse(True, False)
    
    def compassCallback(self, data):
        self.curHeading = data.yaw

    def maniCallback(self, data):
        self.maniData = data.mani_data

    def searchComplete(self):
        if not self.testing:
            self.toMission(search_request=True)

    def abortMission(self):
        if not self.testing:
            self.toMission(fail_request=True, task_complete_request=False)
        self.isAborted = True
        self.isKilled = True
        self.stopRobot()

    def taskComplete(self):
        if not self.testing:
            self.toMission(task_complete_request=True)
        self.isAborted = True
        self.isKilled = True
        self.stopRobot()

    #Perform red thresholding
    def findTheBucket(self, cv_image):
        cv_image = cv2.resize(cv_image, dsize=(self.screen['width'], self.screen['height']))
        cv_image = cv2.GaussianBlur(cv_image, ksize=(5, 5), sigmaX=0)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #Convert to HSV image

        #Perform red thresholding
        contourImg = cv2.inRange(hsv_image, self.lowThresh1, self.hiThresh1)
        
        #Noise removal
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        
        contourImg = cv2.erode(contourImg, erodeEl)
        contourImg = cv2.dilate(contourImg, dilateEl)
        contourImg = cv2.morphologyEx(contourImg, cv2.MORPH_OPEN, openEl)

        out = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2BGR)
      
        #Find centroid
        pImg = contourImg.copy()
        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        maxArea = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.areaThresh and area > maxArea:
                maxArea = area
                maxContour = contour

                # Find center with moments
                mu = cv2.moments(contour, False)
                mu_area = mu['m00']
                centroidx = mu['m10'] / mu_area
                centroidy = mu['m01'] / mu_area
                
                self.rectData['area'] = area
                self.rectData['centroid'] = (centroidx, centroidy)
                self.rectData['rect'] = cv2.minAreaRect(contour)
    
        #Find the largest rect area
        if maxArea > 0: 
            self.rectData['detected'] = True
            
            #Testing
            centerx = int(self.rectData['centroid'][0])
            centery = int(self.rectData['centroid'][1])
            contourImg = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2RGB)
            cv2.circle(out, (centerx, centery), 5, (0, 255, 0))
            cv2.drawContours(out, np.array([maxContour]), 0, (0, 0, 255), 3) 
        else:
            self.rectData['detected'] = False 
              
        return out 
              
    def cameraCallback(self, ros_image):
        cv_image = self.rosimg2cv(ros_image)
        
        centroid_image = self.findTheBucket(cv_image)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(centroid_image, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e) 

if __name__ == "__main__":
    rospy.init_node("bucket_vision")
    bucketDector = BucketDetector()
    rospy.spin()
