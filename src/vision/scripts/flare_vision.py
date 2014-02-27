#!/usr/bin/env python

# Flare computer vision task

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server as DynServer
import vision.cfg.flareConfig as Config

import roslib; roslib.load_manifest('vision')
import rospy
import actionlib
import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError

import collections
import math
import numpy as np
import signal

class Flare:
    #yellow_params = {'lowerH': 56, 'lowerS': 0, 'lowerV': 80, 'higherH': 143, 'higherS':255, 'higherV':240 } 
    highThresh = np.array([143, 255, 255])
    lowThresh = np.array([47, 43, 35])
#     highThresh = np.array([100,161,234])
#     lowThresh = np.array([77,0,210])

    rectData = {'detected': False, 'centroids': (0,0), 'rect': None, 'angle': 0.0, 'area':0, 'length':0,
                'width':0, 'aspect':0.0}
    previous_centroids = collections.deque(maxlen=7)
    areaThresh = 800
    
    bridge = None
    
    curHeading = 0.0
    depth_setpoint = 0.2
    yaw = 0.0
        
    screen = {'width': 640, 'height': 480}

    deltaXMultiplier = 15.0
    sidemoveMovementOffset = 0.3    #For sidemove plus straight
    forwardOffset = 0.3     #For just shooting straight
    headOnArea = 5000       #Area for shooting straight
    
    #Necessary publisher and subscribers
    image_pub = None
    image_sub = None
    yaw_sub = None
    locomotionClient = actionlib.SimpleActionClient("LocomotionServer", bbauv_msgs.msg.ControllerAction)
    
    '''
    Flare Node vision methods
    '''
    def __init__(self):
        self.isAborted = False
        self.isKilled = False
        self.testing = rospy.get_param("~testing", False)
        self.image = rospy.get_param("~image", "/front_camera/camera/image_color")
        
        #Handle signal
        signal.signal(signal.SIGINT, self.userQuit)
        
        self.bridge = CvBridge()
        self.register()
        rospy.loginfo("Flare ready")
        
        # Dynamic reconfigure for flare
        self.dyn_reconf_server = DynServer(Config, self.reconfigure)
        
        #Initialise mission planner communication server and client
        if not self.testing:
            self.isAborted = True
            self.comServer = rospy.Service("/flare/mission_to_vision", mission_to_vision, self.handleSrv)
            self.toMission = rospy.ServiceProxy("/flare/vision_to_mission", vision_to_mission)
            self.toMission.wait_for_service(timeout=5)
            
        #Initialising controller service
        controllerServer = rospy.ServiceProxy("/set_controller_srv",set_controller)
        if self.testing:
            controllerServer(forward=True, sidemove=True, heading=True, depth=False, pitch=False, roll=False,
                             topside=False, navigation=False)
        else:
            controllerServer(forward=True, sidemove=True, heading=True, depth=True, pitch=False, roll=False,
                 topside=False, navigation=False)
        
        #Make sure locomotion server up
        try:
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.logerr("Locomotion server timeout!")
            self.isKilled = True
            
        rospy.loginfo("Flare ready")
        
    def userQuit(self, signal, frame):
        self.isAborted = True
        self.isKilled = True
        rospy.signal_shutdown("Bye!")
            
    def reconfigure(self, config, level):
        rospy.loginfo("Got reconfigure request")
        self.areaThresh = config['area_thresh']
         
#         self.lowThresh[0] = config['lowH']
#         self.lowThresh[1] = config['lowS']
#         self.lowThresh[2] = config['lowV']
#         self.highThresh[0] = config['hiH']
#         self.highThresh[1] = config['hiS']
#         self.highThresh[2] = config['hiV']
#         self.deltaXMultiplier = config['deltaX_multiplier']
#         self.sidemoveMovementOffset = config['sidemove_movement_offset']
#         self.forwardOffset = config['forward_offset']
#         self.headOnArea = config['head_on_area']       
        
        return config
    
    def register(self):
        self.image_pub = rospy.Publisher("/Vision/image_filter" , Image)
        self.image_sub = rospy.Subscriber(self.image, Image, self.camera_callback)
#         self.image_sub = rospy.Subscriber("/front_camera/camera/image_rect_color_opt", Image, self.camera_callback)
        self.yaw_sub = rospy.Subscriber('/euler', compass_data, self.yaw_callback)
        rospy.loginfo("Topics registered")
        
    def unregister(self):
        self.image_pub.unregister()
        self.image_sub.unregister()
        self.yaw_sub.unregister()
        rospy.loginfo("Topics unregistered")
        
    # Handle srv
    def handleSrv(self, req):
        if req.start_request:
            self.isAborted = False
            self.depth_setpoint = req.start_ctrl.depth_setpoint
        elif req.abort_request:
            self.isAborted = True
        return mission_to_visionResponse(True, False)

    def failedTask(self):
        if not self.testing:
            self.toMission(fail_request=True, task_complete_request=False)

    def taskComplete(self):
        if not self.testing:
            #pass
            self.toMission(task_complete_request=True)
        self.sendMovement(forward=-0.5)     #Retract
        self.stopRobot()
        self.isAborted = True
        self.isKilled = True

    def stopRobot(self):
        self.sendMovement(forward=0.0, sidemove=0.0)
    
    #Utility functions to process callback
    def camera_callback(self, image):
        rospy.loginfo("Camera callback")
        out_image = self.findTheFlare(image)
        #self.image_pub.publish(image)

        try:
            if (out_image != None):
                try:
                    out_image = cv2.cv.fromarray(out_image)
                except Exception,e:
                    out_image = np.zeros((self.screen['height'], self.screen['width'], 3), np.uint8)
                if (self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(out_image, encoding="bgr8"))
#                     self.image_pub.publish(self.bridge.cv_to_imgmsg(out_image, encoding="8UC1"))
        except CvBridgeError, e:
            rospy.logerr(str(e))
              
    def yaw_callback(self, msg):
        self.curHeading = msg.yaw
    
    #Utility functions to send movements through locomotion server
    def sendMovement(self, forward=0.0, heading=None, sidemove=0.0, depth=0.4):
#         pass
        depth = depth if depth else self.depth_setpoint
        heading = heading if heading else self.curHeading
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=forward, heading_setpoint=heading,
                                             sidemove_setpoint=sidemove, depth_setpoint=depth)
        rospy.loginfo("forward: {} heading: {} sidemove: {}".format(forward, heading, sidemove))

        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(1))
        
        
    def abortMission(self):
        #Receive mission planner service
        if not self.testing:
            try:
                abortRequest = rospy.ServiceProxy("/flare/vision_to_mission",
                                                  vision_to_mission)
                result = abortRequest(task_complete_request=True)
                #rospy.loginfo(str(result))
            except rospy.ServiceException, e:
                pass
        self.stopRobot()
        self.isAborted = True
        self.isKilled = True
        
    
    def findTheFlare(self, image):
        rospy.loginfo("Finding flare")
        #Convert ROS to CV image 
        try:
            cv_image = self.rosimg2cv(image)
        except CvBridgeError, e:
            rospy.logerr(str(e))
        out = cv_image.copy()                                   #Copy of image for display later
        #cv_image = cv2.merge(np.array([cv2.equalizeHist(cv_image[:,:,0]),cv2.equalizeHist(cv_image[:,:,1]),
        #                               cv2.equalizeHist(cv_image[:,:,2])]))
        cv_image = cv2.resize(cv_image, dsize=(self.screen['width'], self.screen['height']))        
        cv_image = cv2.GaussianBlur(cv_image, ksize=(5, 5), sigmaX=0)
        
        cv_image = cv_image*2

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        contourImg = gray
        ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        fg = cv2.erode(thresh, None, iterations=3)
        bgt = cv2.dilate(thresh, None, iterations=3)
        ret, bg = cv2.threshold(bgt,1,128,1)
        marker = cv2.add(fg,bg)
        marker32 = np.int32(marker)
        cv2.watershed(cv_image, marker32)
        m = cv2.convertScaleAbs(marker32)
        ret,thresh = cv2.threshold(m,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)        
        contourImg = cv2.bitwise_and(cv_image,cv_image,mask = thresh)
               
        contourImg = cv2.cvtColor(contourImg, cv2.COLOR_BGR2GRAY)
          
#         r = np.zeros((self.screen['width'], self.screen['height']), np.uint8)        
#         g = np.zeros((self.screen['width'], self.screen['height']), np.uint8)        
#         cv_image = cv2.merge((contourImg, r, g))


        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)   #Convert to HSV image
        hsv_image = np.array(hsv_image, dtype=np.uint8)         #Convert to numpy array
   
        #Perform yellow thresholding
        contourImg = cv2.inRange(hsv_image, self.lowThresh, self.highThresh)
          
        #Noise removal       
        #contourImg = cv2.morphologyEx(contourImg, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
  
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
        contourImg = cv2.dilate(contourImg, dilateEl)
        contourImg = cv2.erode(contourImg, erodeEl, iterations=1)
       
        #Find centroids
        pImg = contourImg.copy()
        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
   
        rectList = []
        for contour in contours:
            rectData = {}
            area = cv2.contourArea(contour)
            if area > self.areaThresh:
                # Find center with moments
                mu = cv2.moments(contour, False)
                mu_area = mu['m00']
                centroidx = mu['m10']/mu_area
                centroidy = mu['m01']/mu_area
                   
                rectData['area'] = area
                rospy.loginfo("Area: {}".format(rectData['area']))
                rectData['centroids'] = (centroidx, centroidy)
                rectData['rect'] = cv2.minAreaRect(contour)
       
                points = np.array(cv2.cv.BoxPoints(rectData['rect']))
                 
                #Find angle
                edge1 = points[1] - points[0]
                edge2 = points[2] - points[1]
                if cv2.norm(edge1) > cv2.norm(edge2):
                    edge1[1] = edge1[1] if edge1[1] is not 0 else 0.01
                    rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))
                else:
                    edge2[1] = edge2[1] if edge2[1] is not 0 else 0.01
                    rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))
               
                epislon = 10.0
                if -epislon < rectData['angle'] < epislon:
                    rectData['length'] = max(self.calculateLength(points[0], points[1]),
                                                  self.calculateLength(points[1], points[2]))
                    rectData['width'] = min(self.calculateLength(points[0], points[1]),
                                                  self.calculateLength(points[1], points[2]))
                    rectData['aspect'] = rectData['length']/rectData['width']
                          
#                     #Find the median of the last four
#                      if self.previous_centroids:
#                          x_median, y_median = np.median(self.previous_centroids, axis=0)
#                          if abs(rectData['centroids'][0]-x_median)< 0.3 and abs(rectData['centroids'][1]-y_median)<0.3:
                    rectList.append(rectData)                            
                    self.previous_centroids.append(rectData['centroids'])
          
        #Find the largest rect length
        rectList.sort(cmp=None, key=lambda x: x['aspect'], reverse=True)
        if rectList:
            self.rectData = rectList[0]
            self.rectData['detected'] = True
            rospy.loginfo("Angle: {}".format(self.rectData['angle']))            
               
            #Draw output image 
            centerx = int(self.rectData['centroids'][0])
            centery = int(self.rectData['centroids'][1])
            #x_median, y_median = np.mean(self.previous_centroids, axis=0)
            #centerx = int(x_median)
            #centery = int(y_median) 
            contourImg = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2RGB)
            cv2.circle(contourImg, (centerx, centery), 5, (255,0,0))
            cv2.circle(out, (centerx, centery), 5, (255,255,255))
            for i in range (4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                                 
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
    
    def calculateLength(self, pt1, pt2):
        return (pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2
    
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
        
if __name__ == "__main__":
    rospy.init_node("flare_vision")
    flareDetection = Flare()
    rospy.spin()