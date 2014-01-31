## Flare vision class

from bbauv_msgs.msg import compass_data
from bbauv_msgs.msg import controller
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
    
    isCenteringState = False
    isAlignState = True
    
    bridge = None
    image_speed = None
    
    yaw = 0
    centroidx = 0
    centroidy = 0
    centroidx_list = None
    centroidy_list = None
    angleList = None
    max_area = 0
    
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
    def __init__(self, debug_state):
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
          contourRectList = list()
          self.angleList = list()
          max_area = 0
          areaThresh = 3000
          maxRect = None
          
          if (contours != None):
              for i in range (0, len(contours)):
                  #Find the center using moments 
                  mu = cv2.moments(contours[i], binaryImage=False)
                  humoments = cv2.HuMoments(mu)  

                  mu_area = mu['m00']
                  if (mu_area > 700):
                      #center_max.x = mu['m10']/mu_area
                      #center_max.y = mu['m01']/mu_area
                      #max_area = area
                                            
                      #Find bounding rect 
                      cv2.drawContours(contourImg, contours, i, (100, 255, 100), lineType=8, thickness=1, maxLevel=0)
                      countourRect = cv2.minAreaRect(contours[i])
                      pos,size,theta = contourRect
                      contourRectList.append(contourRect)
                      
                      #Find the corners of the box
                      points = cv2.cv.BoxPoints(contourRect)
                      longest_pt1 = None
                      longest_pt2 = None
                      longest_norm = 0
                      for i in range(4):
                          pt1 = (int(points[i][0]), int(points[i][1]))
                          pt2 = (int(point[(i+1)%4][0]), int(points[(i+1)%4][1]))
                          norm = math.sqrt(pow((pt1[0]-pt2[0]),2) + pow((pt1[1]-pt2[1]),2))
                          if (norm > longest_norm):
                              longest = longest_norm = norm
                              longest_pt1 = pt1
                              longest_pt2 = pt2
                          cv2.line(contourImg, pt1, pt2, 255, 1)
                          
                      if (abs(humoments[0] - 0.202) < 0.01):
                          if (longest_pt2[1] < longest_pt1[1]):
                              temp = longest_pt2        #Swap the two points
                              longest_pt2 = longest_pt1
                              longest_pt1 = temp
                              cv.line(contourImg, longest_pt1, longest_pt2, (0,0,255), 2)
                              rect_y = longest_pt2[1] - longest_pt1[1]
                              rect_x = longest_pt2[0] - longest_pt1[0]
                              angle_hori = math.degrees(math.atan2(rect_y, rect_x))
                              #normalise angle
                              if angle_hori == 0.0:
                                  angle_hori = 90      
                              self.angleList.append(angle_hori)
                      
                      if (self.isAlignState):
                          centroidx.append(center_max.x)
                          centroidy.append(center_max.y)
                          cv2.circle(contourImg, (int(centroidx[len(centroidx)-1]), int(centroidy[len(centroidy)-1])), 2, (0,0,255), thickness=1)
                          #TODO: CHANGE THE COLOURS OF THE BOX AND CIRCLE DRAWN TO SUIT ACTUAL IMAGE
                          
          else:
              self.centroidx = 0  
              self.centroidy = 0
                  
          #Central centroid for veicle centering
          if len(centroidx) > 0:
              self.centroidx = centroidx
              self.centroidy = centroidy
              self.centroidx, self.centroidy = self.computeCenter(centroidx, cenroidy)
              cv2.circle(contourImg, (int(self.centroidx), int(self.centroidy)),2,(0,0,255), thickness=1)
          else:
              self.centroidx = 0
              self.centroidy = 0
              
          #Compute angle correction for vehicle orientation
          if len(self.angleList) > 1:
              self.isCenteringState = True
              self.orientation = self.angleList[1]
          elif len(self.angleList) == 1:
              self.isCenteringState = True
              self.orientation = self.angleList[0]
          else:
              self.isCenteringState = False
          if self.orientation != None:
              cv2.putText(contourImg, str(np.round(self.orientation,1)) + " " + str(np.round(self.yaw,1)), 
                          (int(self.centroidx), int(self.centroidy)), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))    
              
          #pos, size, theta
          if len(contourRectList) > 0:
              for rect in contourRectList:
                  temp_area = int(rect[1][0]) * int(rect[1][1])
                  if temp_area > max_area:
                      max_area = temp_area
              self.max_area = max_area
              
              
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
        
        centroid_image = self.findTheFlare(hsv_image)
        
        try:
            if (centroid_image != None):
                centroid_image = cv2.cv.fromarray(centroid_image)
                if (self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(centroid_image, encoding="bgr8"))
        except CvBridgeError, e:
            print e
            
if __name__ == '__main__':
    rospy.init_node('Flare Vision', anonymous=False)
    rosRate = rospy.Rate(20)
    fv = flarevision(False)
    rospy.loginfo("Flare loaded!")
    
    register()
    img = findTheFlare()
    processImage(img)    
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down flare")
    pass
        