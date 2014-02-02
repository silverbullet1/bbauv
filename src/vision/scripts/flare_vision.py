#!/usr/bin/env python

# Flare computer vision task

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import roslib; roslib.load_manifest('vision')
import rospy
import actionlib
import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np
import signal

class Flare:
    yellow_params = {'lowerH': 56, 'lowerS': 105, 'lowerV': 50, 'higherH': 143, 'higherS':251, 'higherV':255 } 
    rectData = {'detected': False, 'centroids': (0,0), 'rect': None, 'angle': 0.0, 'area':0}
    areaThresh = 1000
    
    bridge = None
    image_topic = None
    
    curHeading = 0.0
    depth_setpoint = 0.2
    yaw = 0.0
    
    isAborted = False
    screen = {'width': 640, 'height': 480}
    
    #Necessary publisher and subscribers
    image_pub = None
    image_sub = None
    yaw_sub = None
    locomotionClient = actionlib.SimpleActionClient("LocomotionServer", bbauv_msgs.msg.ControllerAction)
    
    '''
    Flare Node vision methods
    '''
    def __init__(self):
#         self.isAborted = True
#         self.isKilled = False
#         #Handle signal
#         signal.signal(signal.SIGINT, self.userQuit)
        
        self.image_topic = rospy.get_param('~image', '/front_camera/camera/image_rect_color')
        #TODO: Add histogram modes for debug
        self.bridge = CvBridge()
        self.register()
        rospy.loginfo("Flare ready")
        
        #Initialise mission planner communication server and client
#         rospy.loginfo("Waiting for mission_to_vision server...")
#         try:
#             rospy.wait_for_service("/flare/mission_to_vision", timeout=5)
#         except:
#             rospy.kklogerr("mission_to_vision timeout!")
#         self.comServer = rospy.Service("/flare/mission_to_vision", mission_to_vision, self.handleSrv)
#         
#         try:
#             rospy.loginfo("Waiting for Locomotion Server", timeout=5)
#             self.locomotionClient.wait_fo_server()
#         except:
#             rospy.loginfo("Locomotion Server timeout!")
        
        def userQuit(self, signal, frame):
            self.isAborted = True
            self.isKilled = True
            
    def register(self):
        self.image_pub = rospy.Publisher("/Vision/image_filter_opt/" , Image)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.camera_callback)
        self.yaw_sub = rospy.Subscriber('/euler', compass_data, self.yaw_callback)
        rospy.loginfo("Topics registered")
        
    def unregister(self):
        self.image_pub.unregister()
        self.image_sub.unregister()
        self.yaw_sub.unregister()
        rospy.loginfo("Topics unregistered")
    
    #Utility functions to process callback
    def camera_callback(self, image):
        out_image = self.findTheFlare(image)
        #self.image_pub.publish(image)


        try:
            if (out_image != None):
                out_image = cv2.cv.fromarray(out_image)
                if (self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(out_image, encoding="rgb8"))
                    #self.image_pub.publish(self.bridge.cv_to_imgmsg(out_image, encoding="8UC1"))
        except CvBridgeError, e:
            print e
              
    def yaw_callback(self, msg):
        self.yaw = msg.yaw
    
    #Utility functions to send movements through locomotion server
    def sendMovement(self, forward=0.0, heading=None, sidemove=0.0, depth=None):
        depth = depth if depth else self.depth_setpoint
        heading = heading if heading else self.curHeading
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=forward, heading_setpoint=heading,
                                             sidemove_setpoint=sidemove, depth_setpoint=depth)
        self.locomotionClient.send_goal(goal)
        
    def abortMission(self):
        self.isAborted = True
    
    def findTheFlare(self, image):
        #Convert ROS to CV image 
        try:
            cv_image = self.rosimg2cv(image)
        except CvBridgeError, e:
            print e
        out = cv_image.copy()                                   #Copy of image for display later
        cv_image = cv2.GaussianBlur(cv_image, ksize=(5, 5), sigmaX=0)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)   #Convert to HSV image
        hsv_image = np.array(hsv_image, dtype=np.uint8)         #Convert to numpy array

        #Perform yellow thresholding
        lowerBound = np.array([self.yellow_params['lowerH'], self.yellow_params['lowerS'], self.yellow_params['lowerV']],np.uint8)
        higherBound = np.array([self.yellow_params['higherH'], self.yellow_params['higherS'], self.yellow_params['higherV']],np.uint8)
        contourImg = cv2.inRange(hsv_image, lowerBound, higherBound)
        
        #Noise removal
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (13,13))
        contourImg = cv2.erode(contourImg, erodeEl)
        contourImg = cv2.dilate(contourImg, dilateEl)
      
        #Find centroids
        pImg = contourImg.copy()
        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        rectList = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.areaThresh:
                # Find center with moments
                mu = cv2.moments(contour, False)
                mu_area = mu['m00']
                centroidx = mu['m10']/mu_area
                centroidy = mu['m01']/mu_area
                
                self.rectData['area'] = area
                self.rectData['centroids'] = (centroidx, centroidy)
                self.rectData['rect'] = cv2.minAreaRect(contour)
    
                points = np.array(cv2.cv.BoxPoints(self.rectData['rect']))
              
                #Find angle
                edge1 = points[1] - points[0]
                edge2 = points[2] - points[1]
                if cv2.norm(edge1) > cv2.norm(edge2):
                    edge1[1] = edge2[1] if edge2[1] is not 0 else 0.01
                    self.rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))
                else:
                    edge1[1] = edge2[1] if edge2[1] is not 0 else 0.01
                    self.rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))
            
                rospy.loginfo(self.rectData['angle'])

                if -10 < self.rectData['angle'] < 10:                    
                    rectList.append(self.rectData)
        
        #Find the largest rect area
        rectList.sort(cmp=None, key=lambda x: x['area'], reverse=True)
        if rectList:
            self.rectData = rectList[0]
            self.rectData['detected'] = True
            
            #Draw output image 
            centerx = int(self.rectData['centroids'][0])
            centery = int(self.rectData['centroids'][1])
            contourImg = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2RGB)
            cv2.circle(contourImg, (centerx, centery), 5, (255,0,0))
            cv2.circle(out, (centerx, centery), 5, (255,255,255))
            for i in range (4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                length = int(points[i][0]) - int(points[(i+1)%4][0])
                width = int(points[i][1]) - int(points[(i+1)%4][1])
                #print length
                #print width
                cv2.line(contourImg, pt1, pt2, (255,0,0))
                cv2.line(out, pt1, pt2, (0,0,255))
            cv2.putText(out, str(self.rectData['angle']), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
            cv2.putText(contourImg, str(self.rectData['angle']), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
            
        else:
            self.rectData['detected'] = False 
            contourImg = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2RGB)            
              
        #return out
        return contourImg
       
    #Convert ROS image to Numpy matrix for cv2 functions 
    def rosimg2cv(self, ros_image):
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        return np.array(frame, dtype = np.uint8)
    
    def processImage(self, data):
        try:
            cv_image = self.rosimg2cv(data)
        except CvBridgeError, e:
            print e
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)   #Convert to HSV image
        hsv_image = np.array(hsv_image, dtype=np.uint8)         #Convert to numpy array
        
        return hsv_image
        