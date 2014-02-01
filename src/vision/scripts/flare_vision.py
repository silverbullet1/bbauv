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
    rectData = {'detected': False, 'centroids': (0,0), 'rect': None}
    areaThresh = 3000
    
    bridge = None
    image_topic = None
    
    curHeading = 0.0
    depth_setpoint = 0.2
    yaw = 0.0
    
    isAborted = False
    
    #Necessary published methods 
    image_pub = None
    image_sub = None
    yaw_sub = None
    locomotionClient = actionlib.SimpleActionClient("LocomotionServer", bbauv_msgs.msg.ControllerAction)
        
    '''
    Utility Methods 
    '''
    
    #Convert ROS image to Numpy matrix for cv2 functions 
    def rosimg2cv(self, ros_image):
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        return np.array(frame, dtype = np.uint8)
    
    '''
    Flare Node vision methods
    '''
    def __init__(self, debug_state):
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
        
        try:
            if (centroid_image != None):
                centroid_image = cv2.cv.fromarray(centroid_image)
                if (self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(centroid_image, encoding="bgr8"))
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
              
          return contourImg
              
    def computeCenter(self, centroid_x, centroid_y):
          x_ave = np.ave(centroid_x, None, None)
          y_ave = np.ave(centroid_y, None, None)
          return x_ave, y_ave
          
    def processImage(self, data):
        try:
            cv_image = self.rosimg2cv(data)
        except CvBridgeError, e:
            print e
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)   #Convert to HSV image
        hsv_image = np.array(hsv_image, dtype=np.uint8)         #Convert to numpy array
        
        return hsv_image
        