
'''
###################################################################

                       COMPUTER VISION CLASS
        
###################################################################
'''
from com.histogram.histogram import Hist_constants
from com.histogram.histogram import bbHistogram
from com.shape.ShapeAnalysis import ShapeAnalysis
from bbauv_msgs.msg import *
from bbauv_msgs.srv import * 

import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import rospy
import math

#External libraries
import numpy as np

class SpeedTrap:
    debug = True
    red_params = {'hueLow': 0, 'hueHigh':10,'satLow': 100, 'satHigh': 255,'valLow':0,'valHigh':255,'topHueLow':170,'topHueHigh':180}
    yellow_params = {'hueLow': 20, 'hueHigh':60,'satLow': 0, 'satHigh': 255,'valLow':0,'valHigh':255}
    yellow_hist = None
    red_hist = None
    isAlignState = True
    isLoweringState = True
    isAim = False
    isCentering = False
    #shapeClass = ShapeAnalysis()
    yaw = 0
    aim_point = None
    centroidx = 0
    centroidy = 0
    rows = 0
    cols = 0
    binList = None
    orientation = None
    #params = { 'satLow': 50, 'satHigh': 255, 'hueLow': 36, 'hueHigh':48,'valLow':0,'valHigh':255,'Kp':10,'Vmax':40 }
    bridge = None
    position = None
    centroidx_list = None
    centroidy_list = None
    angleList = list()
    max_area = 0
    outer_center = 40
    inner_center = 20
    counter = 0
    ''' 
    Utility Methods
    '''
    
    def paramSetter(self,key):
        def setter(val):
            self.params[key] = val
        return setter    
    
    def stParamSetter(self,key):
        def setter(val):
            self.stParams[key] = val
        return setter    
  
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image trackbarnameto Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
    
    def maskROI(self,image,rect):
        maskImage = np.copy(image)
        row, col= maskImage.shape
        x = rect[0]
        y = rect[1]
        maskImage[0:y] = 0
        maskImage[0:x] = 0
        maskImage[y + rect[3]:row] = 0
        maskImage[x+ rect[2]:col] = 0
        return maskImage
    
    '''
    Node Functions
    '''    
    def __init__(self,debug_state):
        #imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_rect_color')
        #yawTopic = rospy.get_param('~compass', '/euler')
        self.debug = debug_state
        if self.debug:
            yellow_hist = bbHistogram("yellow",Hist_constants.TRIPLE_CHANNEL)
            red_hist = bbHistogram("red",Hist_constants.TRIPLE_CHANNEL)
            self.yellow_hist.setParams(self.yellow_params)
            self.red_hist.setParams(self.red_params)
        self.bridge = CvBridge()
        rospy.loginfo("Speedtrap Ready")
 
    def register(self):
        self.image_pub = rospy.Publisher("/Vision/image_filter",Image)
        self.image_sub = rospy.Subscriber(rospy.get_param('~image','/bottomcam/camera/image_rect_color'), Image,self.processImage)
        self.yaw_sub = rospy.Subscriber('/euler',compass_data,self.collectYaw)
        rospy.loginfo("Topics registered")
    def unregister(self):
        self.image_sub.unregister()
        self.image_pub.unregister()
        self.yaw_sub.unregister()
        rospy.loginfo("Topics unregistered")
        
    def collectYaw(self,msg):
        self.yaw = msg.yaw
    
    def aiming(self,image):
        if self.debug:
            self.red_hist.setParams(self.red_params)
        COLOR_MIN = np.array([self.red_params['hueLow'],self.red_params['satLow'],self.red_params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.red_params['hueHigh'],self.red_params['satHigh'],self.red_params['valHigh']],np.uint8)
        
        bot_red_image = cv2.inRange(image,COLOR_MIN, COLOR_MAX)
        
        COLOR_MIN = np.array([self.red_params['topHueLow'],self.red_params['satLow'],self.red_params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.red_params['topHueHigh'],self.red_params['satHigh'],self.red_params['valHigh']],np.uint8)
        
        top_red_image = cv2.inRange(image,COLOR_MIN, COLOR_MAX)
        
        red_image = top_red_image + bot_red_image
        
        if self.debug:
            self.red_hist.getTripleHist(image)
        '''Find contours on binary image and identify target to home in on'''
        '''Perform Morphological Operations on binary image to clean it up'''
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE,kernel_close,iterations=5)
        if(self.isLoweringState):
            shape_array = cv2.split(image)
            #retval, shape_image = cv2.threshold(shape_array[2], self.params['valLow'], 255, cv2.THRESH_OTSU)
            #self.histClass.setTrackBarPosition("Value Low:", int(retval))
            #retval, red_image = cv2.threshold(shape_array[0], self.red_params['hueLow'], 255, cv2.THRESH_OTSU)
            #self.red_hist.setTrackBarPosition("red_Hue Low:", int(retval))
            contours,hierarchy = cv2.findContours(red_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            contourImg = np.zeros((self.rows,self.cols,3),dtype=np.uint8)
            if(contours != None):
                for i in range(0,len(contours)):
                    moments =cv2.moments(contours[i],binaryImage=False)
                    if moments['m00'] > 50:
                        humoments = cv2.HuMoments(moments)
                        cv2.drawContours(contourImg, contours, i, (0,0,255), lineType=8, thickness= 1,maxLevel=0)
            #shape_image = shape_array[2] - white_image
            #shape_image1 = np.copy(shape_image)
            #self.shapeClass.predictSurf(shape_array[2])
        return contourImg
    
    def centroidIdentification(self,image):
        
        '''
        
            PERFORM YELLOW THRESHOLDING ON IMAGE
        
        '''
        #self.yellow_param = self.yellow_hist.getParams()
        if self.debug:
            self.yellow_hist.setParams(self.yellow_params)
        #COLOR_MIN = np.array([self.yellow_params['hueLow'],self.yellow_params['satLow'],self.yellow_params['valLow']],np.uint8)
        #COLOR_MAX = np.array([self.yellow_params['hueHigh'],self.yellow_params['satHigh'],self.yellow_params['valHigh']],np.uint8)
        imgArray = cv2.split(image)
        
        #retval, cv_thres = cv2.threshold(imgArray[0], self.yellow_params['hueLow'], 255, cv2.THRESH_OTSU)
        #self.yellow_hist.setTrackBarPosition("yellow_Hue Low:", int(retval))
        retval = 91
        if retval > 90:
            hue_low = self.yellow_params['hueLow']
            hue_high = self.yellow_params['hueHigh']
        else:
            hue_low = 0
            hue_high = 0
        
        COLOR_MIN = np.array([hue_low,self.yellow_params['satLow'],self.yellow_params['valLow']],np.uint8)
        COLOR_MAX = np.array([hue_high,self.yellow_params['satHigh'],self.yellow_params['valHigh']],np.uint8)
        cv_single = cv2.inRange(image,COLOR_MIN, COLOR_MAX)
        
        if self.debug:
            self.yellow_hist.getTripleHist(image)
        if retval < 90:
            cv_single = cv_single + cv_thres
            
        '''Find contours on binary image and identify target to home in on'''
        '''Perform Morphological Operations on binary image to clean it up'''
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
        cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_CLOSE,kernel_close,iterations=1)
        contours,hierarchy = cv2.findContours(cv_single, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.rows,self.cols,val = image.shape
        contourImg = np.zeros((self.rows,self.cols,3),dtype=np.uint8)
        centroidx = list()
        centroidy = list()
        binList = list()
        self.angleList = list()
        if(contours != None):
            for i in range(0,len(contours)):
                moments =cv2.moments(contours[i],binaryImage=False)
                if moments['m00'] > 700:
                    humoments = cv2.HuMoments(moments)
                    cv2.drawContours(contourImg, contours, i, (100,255,100), lineType=8, thickness= 1,maxLevel=0)
                    contourRect= cv2.minAreaRect(contours[i])
                    pos,size,theta = contourRect
                    binList.append(contourRect)
                    # Obtain the actual corners of the box
                    points = cv2.cv.BoxPoints(contourRect)
                    longest_pt1 = None
                    longest_pt2 = None
                    longest_norm = 0
                    # Draw the lines
                    for i in range(4):
                        # The line function doesn't accept floating point values
                        pt1 = (int(points[i][0]), int(points[i][1]))
                        pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                        norm = math.sqrt(pow((pt1[0] - pt2[0]),2) + pow((pt1[1] - pt2[1]),2))
                        if (norm > longest_norm):
                            longest = i
                            longest_norm = norm
                            longest_pt1 = pt1
                            longest_pt2 = pt2
                        cv2.line(contourImg, pt1, pt2, 255, 1)
                    if(abs(humoments[0] - 0.202) < 0.01):
                        if(longest_pt2[1] < longest_pt1[1]):
                            temp = longest_pt2
                            longest_pt2 = longest_pt1
                            longest_pt1 = temp 
                        cv2.line(contourImg, longest_pt1, longest_pt2, (0,0,255), 2)
                        rect_y = longest_pt2[1] - longest_pt1[1]
                        rect_x = longest_pt2[0] - longest_pt1[0]
                        angle_hor = math.degrees(math.atan2(rect_y,(rect_x)))
                        if angle_hor == 0.0:
                            angle_hor = 90 
                        self.angleList.append(angle_hor)
                        cv2.putText(contourImg,str(np.round(angle_hor,1)), (int(points[0][0]),int(points[0][1])), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
                    colorTxt = (0,0,255)
                    #cv2.drawContours(contourImg, contours, i, (255,0,0),thickness= -1)
                    if(self.isAlignState):
                        centroidy.append(moments['m01']/moments['m00'])
                        centroidx.append(moments['m10']/moments['m00'])
                        cv2.circle(contourImg,(int(centroidx[len(centroidx) -1]),int(centroidy[len(centroidy) - 1])), 2, (0,0,255), thickness=-1)
        else:
            self.centroidx = 0
            self.centroidy = 0
        # Compute Central centroid for vehicle centering
        if len(centroidx) > 0:
            self.centroidx_list = centroidx
            self.centroidy_list = centroidy
            self.centroidx, self.centroidy = self.computeCenter(centroidx, centroidy) 
            cv2.circle(contourImg,(int(self.centroidx),int(self.centroidy)), 2, (0,0,255), thickness=-1)
        else:
            self.centroidx = 0
            self.centroidy = 0
        #Compute orientation angle correction for vehicle orientation
        if len(self.angleList) >1:
            self.isCentering = True
            final_list = sorted(self.angleList)
            self.orientation = self.angleList[1]
        elif len(self.angleList) == 1:
            self.isCentering = True
            self.orientation = self.angleList[0]
        else:
            self.isCentering = False
            
        #pos,size,theta
        max_area = 0
        if len(binList) > 0:
            for rect in binList:
                temp_area = int(rect[1][0])*int(rect[1][1])
                if temp_area > max_area:
                    max_area = temp_area
            self.max_area = max_area
        #Display orientation angles
        if self.orientation != None:
            cv2.putText(contourImg,str(np.round(self.orientation,1)) + " " + str(np.round(self.yaw,1)), (int(self.centroidx),int(self.centroidy)), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        return contourImg
    
    def computeCenter(self,centroid_x,centroid_y):
       x_ave = np.average(centroid_x,None,None)
       y_ave = np.average(centroid_y,None,None)
       return x_ave,y_ave
   
    def draw_crosshair(self,image, center,radius):
        cv2.line(image, (center[0] - radius,center[1]),(center[0] + radius,center[1]), (0,0,255))
        cv2.line(image, (center[0],center[1] - radius),(center[0],center[1] + radius), (0,0,255))
        return image
    
    def draw_aiming_box(self,image,center,space,color):
        cv2.rectangle(image,(center[0] - space,center[1] - space),(center[0] + space,center[1] + space), color)
        return image
    def processImage(self,data): 
        try:
            cv_image = self.rosimg2cv(data)
        except CvBridgeError, e:
            print e
        #cv_single = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        #cv_single = np.array(cv_single,dtype=np.uint8)
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        hsv_image = np.array(hsv_image,dtype=np.uint8)
        centroid_image = self.centroidIdentification(hsv_image)
        #Draw aiming window on image to center
        color = (255,100,255)
        if(self.cols != None):
            centroid_image = self.draw_crosshair(centroid_image, (self.cols/2, self.rows/2), self.inner_center)
            if self.counter == 0:
                centroid_image = self.draw_crosshair(centroid_image, (self.cols/2 - 150, self.rows/2), self.outer_center)
            elif self.counter == 1:
                centroid_image = self.draw_crosshair(centroid_image, (self.cols/2 + 150, self.rows/2), self.outer_center)        
        if self.aim_point != None:
            cv2.circle(centroid_image,self.aim_point, 2, (0,255,9), thickness=-1)
        if self.isAim:
            shape_image = self.aiming(hsv_image)
        else:
            shape_image = np.zeros((self.rows,self.cols,3),dtype=np.uint8)
        final_image = shape_image + centroid_image
        try:
            if(final_image != None):
                final_image= cv2.cv.fromarray(final_image)
                if(self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(final_image,encoding="bgr8"))
        except CvBridgeError, e:
            print e
