## Flare vision class

from bbauv_msgs.msg import compass_data
from bbauv_msgs.msg import controller
from bbauv_msgs.srv import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import rospy
import math
import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Flare:
    debug = True
    yellow_params = {'lowerH': 10, 'lowerS': 0, 'lowerV': 0, 'higherH': 79, 'higherS':148, 'higherV':255 }
    yellow_hist = None
    isAim = False
    isCentering = False
    
    bridge = None
    image_speed = None
    
    yaw = 0
    centroidx = 0
    centroidy = 0
    centroidx_list = None
    centroidy_list = None
    angleList = None
    counter = 0                 #Counter for number of times the image is being processed
    
    #Necessary published methods 
    image_pub = None
    image_sub = None
    yaw_sub = None
        
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
    def __init__(selfself, debug_state):
        self.debug = debug_state
        self.image_speed = rospy.get_param('~image', '/front_cam/camera/image_rect_color')
        #TODO: Add histogram modes for debug
        self.bridge = CvBridge()
        rospy.loginfo("Flare ready")
            
    def register(self):
        self.image_pub = rospy.Publisher("/front_camera/filter" , Image)
        self.image_sub = rospy.Subscriber(self.image_speed, Image, self.processImage)
        self.yaw_sub = rospy.Subsriber('/euler', compass_data, self.yaw_callback)
        rospy.loginfo("Topics registered")
        
    def unregister(self):
        self.image_pub.unregister()
        self.image_sub.unregister()
        self.yaw_sub.unregister()
        rospy.loginfo("Topics unregistered")
    
    def yaw_callback(self, msg):
        self.yaw = msg.yaw
    
    #Perform yellow thresholding
    def findTheFlare(self, image):
#         if self.debug:
#         TODO: Prepare Histogram to display on control panel

          kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
          kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
          cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE, kernel_close, iterations=1)
          contours, hierachy = cv2.findContours(cv_single, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
          
          rows, cols, val = image.shape
          contourImg = np.zeros((rows, cols, 3), dtype=np.uint8)
          centroidx = list()
          centroidy = list()
          binList = list()
          self.angleList = list()
          max_area = 0
          areaThresh = 3000
          maxRect = None
          
          if (contours != None):
              for i in range (0, len(contours)):
                  area = cv2.contourarea(contours[i])
                  if (area > areaThresh and area > max_area):
                      #Find the center using moments 
                      mu = cv2.moments(contours[i], binaryImage=False)
                      mu_area = mu['m00']
                      center_max.x = mu['m10']/mu_area
                      center_max.y = mu['m01']/mu_area
                      max_area = area
                      
                      #Find bounding rect 
                      maxRect = cv2.minAreaRect(contours[i])
                  
          
          
          
          
        
        