# Flare computer vision task

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import roslib; roslib.load_manifest('vision')
import rospy
import math
import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Flare:
    yellow_params = {'lowerH': 10, 'lowerS': 0, 'lowerV': 0, 'higherH': 79, 'higherS':148, 'higherV':255 } 
    rectData = {'detected': False, 'centroids': (0,0), 'rect': None, 'angle': 0.0}
    areaThresh = 3000
    
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
        self.image_topic = rospy.get_param('~image', '/front_cam/camera/image_rect_color')
        #TODO: Add histogram modes for debug
        self.bridge = CvBridge()
        rospy.loginfo("Flare ready")
            
    def register(self):
        self.image_pub = rospy.Publisher("/front_camera/filter" , Image)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.camera_callback)
        self.yaw_sub = rospy.Subsriber('/euler', compass_data, self.yaw_callback)
        rospy.loginfo("Topics registered")
        
    def unregister(self):
        self.image_pub.unregister()
        self.image_sub.unregister()
        self.yaw_sub.unregister()
        rospy.loginfo("Topics unregistered")
    
    #Utility functions to process callback
    def camera_callback(self, Image):
        hsv_image = self.processImage(Image)
        centroid_image = self.findTheFlare(hsv_image)
        
        self.publishImage(centroid_image)       
    
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
        #Inform mission planner 
    
    
    def findTheFlare(self, image):
          #Perform yellow thresholding
          lowerBound = np.array([yellow_params['lowerH'], yellow_params['lowerS'], yellow_params['lowerV']],np.uint8)
          higherBound = np.array([yellow_params['higherH'], yellow_params['higherS'], yellow_params['higherV']],np.uint8)
          contourImg = cv2.inRange(image, lowerBound, higherBound)
          
          #Noise removal
          erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
          dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
          contourImg = cv2.erode(contourImg, erodeEl)
          contourImg = cv2.dilate(contourImg, dilateEl)
          
          #Find centroids
          pImg = contourImg.copy()
          contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
          
          max_area = 0
          for contour in contours:
              area = cv2.contourArea(contour)
              if area > self.areaThresh and area > maxArea:
                  # Find center with moments
                  mu = cv2.moments(contour, False)
                  mu_area = mu['m00']
                  centroidx = mu['10']/mu_area
                  centroidy = mu['01']/mu_area
                  max_area = area
                  
                  self.rectData['centroids'] = (centroidx, centroidy)
                  self.rectData['rect'] = cv2.minAreaRect(contour)
        
          if max_area > 0:
              self.rectData['detected'] = True
              points = np.array(cv2.cv.BoxPoints(self.rectData['rect']))
              
              #Find angle
              edge1 = points[1] - points[0]
              edge2 = points[2] - points[1]
              if cv2.norm(edge1) > cv2.norm(edge2):
                  self.rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))
              else:
                   self.rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))

              if self.rectData['angle'] == 90:
                  if self.rectData['centroids'][0] > self.screen['width']/2:
                      self.rectData['angle'] = -90
              elif self.rectData['angle'] == -90:
                  if self.rectData['centroids'][1] < self.screen['width']/2:
                      self.rectData['angle'] = 90

              
              #Draw output image 
              out = image.copy()
              centerx = int(self.rectData['centroids'][0])
              centery = int(self.rectData['centroids'][1])
              cv2.circle(out, (centerx, centery), 5, (255,255,255))
              for i in range (4):
                  pt1 = (int(points[i][0]), int(points[i][1]))
                  pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                  cv2.line(out, pt1, pt2, (255,255,255))
              cv2.putText(out, str(self.rectData['angle']), (30,30),
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
              
          else:
              self.rectData['detected'] = False 
                  
          return out
              
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
    
    def publishImage(self, image):
        try:
            if (image != None):
                image = cv2.cv.fromarray(image)
                if (self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(image, encoding="bgr8"))
        except CvBridgeError, e:
            print e
        