#!/usr/bin/env python2
'''
Identify pipe (pizza delivery task)
'''

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point32
from bbauv_msgs.msg import pipe_pose
from bbauv_msgs.msg import compass_data
from cv_bridge import CvBridge, CvBridgeError

from com.histogram.histogram import Hist_constants
from com.histogram.histogram import bbHistogram

import numpy as np
import cv2
import math
from numpy.core.fromnumeric import argmin

class Drive_thru:
    debug = False
    orange_params = {'hueLow': 20, 'hueHigh':60,'satLow': 0, 'satHigh': 255,'valLow':0,'valHigh':255}
    orange_hist = None
    shape_hu = 0.0
    centroid = None
    orientation = 0
    yaw = 0
    l_center_x = 0
    l_center_y = 0
    outer_center = 40
    inner_center = 20
    max_area = 0
    find_times = 0
    image_param = None
    pipe_skeleton_pose = pipe_pose()
    def __init__(self, debug_state):
        self.image_param = rospy.get_param('~image','/bottomcam/camera/image_rect_color')
        self.debug = debug_state
        if self.debug:
            self.orange_hist = bbHistogram("orange",Hist_constants.TRIPLE_CHANNEL)
            self.orange_hist.setParams(self.orange_params)
        self.cvbridge = CvBridge()
        self.cameraInfo_initialized_ = False
        self.pipe_length_total_ = 0.305
        self.pipe_length_little_ = 0.0
        self.find_times_ = 0
        self.rows = 0
        self.cols = 0
        self.find_times_limit_ = 30
        self.pipePose_pub_ = rospy.Publisher('/pipe_pose', pipe_pose)

    def CameraInfoCB(self, info_msg):
        if not self.cameraInfo_initialized_:
            self.camera_info_ = info_msg
            self.cameraInfo_initialized_ = True
    
    def computeAngle(self,pt1,pt2):
        angle = math.degrees(math.atan2((pt2[1] - pt1[1]),(pt2[0] - pt1[0])))
        return angle
    def processImage(self, image_msg):

        try:
            iplimg = self.cvbridge.imgmsg_to_cv(image_msg, image_msg.encoding)
            cvimg = np.array(iplimg, dtype=np.uint8)
        except CvBridgeError, e:
            rospy.logerr('cvbridge exception: ' + str(e))
            return
        hsv_image = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)
        
        if self.debug:
            self.orange_hist.setParams(self.orange_params)
            self.orange_hist.getTripleHist(hsv_image)
            
        COLOR_MIN = np.array([self.orange_params['hueLow'],self.orange_params['satLow'],self.orange_params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.orange_params['hueHigh'],self.orange_params['satHigh'],self.orange_params['valHigh']],np.uint8)
        
        pipe_orange_threshold = cv2.inRange(hsv_image, COLOR_MIN, COLOR_MAX)

        opening_size = 1
        opening_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2*opening_size+1, 2*opening_size+1))
        closing_size = 5
        closing_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2*closing_size+1, 2*closing_size+1))

        pipe_orange_threshold = cv2.morphologyEx(pipe_orange_threshold, cv2.MORPH_CLOSE, closing_element,iterations=3)

        copy_for_contour = pipe_orange_threshold.copy()
        
        self.pipe_skeleton_pose = pipe_pose()
        #self.centroid = None
        copy_for_contour2 = pipe_orange_threshold.copy()
        contours, _ = cv2.findContours(copy_for_contour2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.rows,self.cols,val = cvimg.shape
        contourImg = np.zeros((self.rows,self.cols,3),dtype=np.uint8)
        if(contours != None):
            self.max_area = 0
            for i in range(0,len(contours)):
                moments =cv2.moments(contours[i],binaryImage=False)
                if moments['m00'] > 1500:
                    #Set centroid
                    self.pipe_skeleton_pose.detect_pipe = True
                    if self.max_area < moments['m00']:
                        self.max_area = moments['m00']
                    self.centroid = (int(moments['m10']/moments['m00']),int(moments['m01']/moments['m00']))
                    cv2.circle(contourImg,self.centroid, 2, (0,0,255), thickness=-1)
                    
                    cv2.drawContours(contourImg, contours, i, (100,255,100), lineType=8, thickness= 2,maxLevel=0)
                    humoments = cv2.HuMoments(moments)
                    minArea_rectangle = cv2.minAreaRect(contours[i])
                    if(abs(humoments[0] - self.shape_hu) < 0.03 and np.fabs(minArea_rectangle[1][0]/minArea_rectangle[1][1] - 1) < 0.1):
                        self.find_times+=1
                        rect_points = cv2.cv.BoxPoints(minArea_rectangle)
                        cv2.putText(contourImg,str(np.round(humoments[0],2)), (int(rect_points[0][0]),int(rect_points[0][1])), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
                        for j in range(4):
                            pt1 = tuple(np.int32(rect_points[j]))
                            pt2 = tuple(np.int32(rect_points[(j+1)%4]))
                            cv2.line(contourImg, pt1, pt2, (255,0,0), 2, 8)
                        #test, lowest_pt, second_lowest_pt = self.Calc_pose(rect_points, self.pipe_skeleton_pose)
                        pt1,pt2 = self.compute_lowest_line(rect_points)
                        
                        #Calculate Center of lowest point
                        self.l_center_x = (pt1[0] + pt2[0])/2
                        self.l_center_y = (pt1[1] + pt2[1])/2
                        
                        pt1 = tuple(np.int32(pt1))
                        pt2 = tuple(np.int32(pt2))
                        
                        cv2.line(contourImg, (int(pt1[0]),int(pt1[1])), (int(pt2[0]),int(pt2[1])), (0,0,255), 2, 8)
                        if self.find_times > 30:
                            self.orientation = np.fabs(self.computeAngle(pt1,pt2))
                        cv2.putText(contourImg,str(np.round(self.orientation,2)), (int(rect_points[3][0]),int(rect_points[3][1])), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
                    else:
                        self.find_times-=1
                        
        self.pipe_skeleton_pose.detection_stable = self.find_times_ > self.find_times_limit_
        self.pipePose_pub_.publish(self.pipe_skeleton_pose)
        
        final_image = contourImg
        final_image = self.draw_crosshair(final_image,(self.cols/2,self.rows/2), self.inner_center)
        cv2.waitKey(2)
        try:
            if(final_image != None):
                final_image= cv2.cv.fromarray(final_image)
                if(self.image_pub != None):
                    self.image_pub.publish(self.cvbridge.cv_to_imgmsg(final_image,encoding="bgr8"))
            #cv2.waitKey(1)
        except CvBridgeError, e:
            print e
    
    def draw_crosshair(self,image, center,radius):
        cv2.line(image, (center[0] - radius,center[1]),(center[0] + radius,center[1]), (0,0,255))
        cv2.line(image, (center[0],center[1] - radius),(center[0],center[1] + radius), (0,0,255))
        return image
    def compute_lowest_line(self,rect_points):
        y_points = list()
        x_points = list()
        for point in rect_points:
            x_points.append(point[0])
            y_points.append(point[1])
        max_y_index = np.argmax(y_points)
        max_pt = rect_points[max_y_index]
        y_points.pop(max_y_index)
        max_y2 = np.max(y_points)
        for point in rect_points:
            if point[1] == max_y2:
                max_pt2 = point
        
        return max_pt, max_pt2
        
    '''   
    def Calc_pose(self, rect_points, pipeFrame_pose):
        dist1 = math.sqrt((rect_points[0][0] - rect_points[1][0])*(rect_points[0][0] - rect_points[1][0])+(rect_points[0][1] - rect_points[1][1])*(rect_points[0][1] - rect_points[1][1]))
        dist2 = math.sqrt((rect_points[2][0] - rect_points[1][0])*(rect_points[2][0] - rect_points[1][0])+(rect_points[0][1] - rect_points[1][1])*(rect_points[0][1] - rect_points[1][1]))
        longer_line, shorter_line = (dist1, dist2) if dist1>=dist2 else (dist2, dist1)
        if shorter_line < 0.001:
            return False, (0,0), (0,0)
        if longer_line/shorter_line > 3.0:
            return False, (0,0), (0,0)

        points_border_image = 0
        for i in range(4):
            if rect_points[i][0]<2.0 or rect_points[i][1]<2.0:
                points_border_image += 1
        if points_border_image >= 3:
            return False, (0,0), (0,0)

        # Find the lowest 2 points in the image, to serve as the two ends of the lowest pipeline bar
        if rect_points[0][1] > rect_points[1][1]:
            lowest_point = rect_points[0]
            second_lowest_point = rect_points[1]
        else:
            lowest_point = rect_points[1]
            second_lowest_point = rect_points[0]
        if rect_points[2][1] > lowest_point[1]:
            second_lowest_point = lowest_point
            lowest_point = rect_points [2]
        elif rect_points[2][1] > second_lowest_point[1]:
            second_lowest_point = rect_points[2]
        if rect_points[3][1] > lowest_point[1]:
            second_lowest_point = lowest_point
            lowest_point = rect_points [3]
        elif rect_points[3][1] > second_lowest_point[1]:
            second_lowest_point = rect_points[3]

        length_pixel_ratio = self.pipe_length_total_ / longer_line
        estimated_depth = length_pixel_ratio * self.camera_info_.K[0]

        realPt1 = self.imagePt_to_realPt(lowest_point, length_pixel_ratio, estimated_depth)
        realPt2 = self.imagePt_to_realPt(second_lowest_point, length_pixel_ratio, estimated_depth)
        if realPt1.y > realPt2.y:
            realPt1, realPt2 = realPt2, realPt1

        pipeFrame_pose.points.append(realPt1)
        pipeFrame_pose.points.append(realPt2)

        return True, lowest_point, second_lowest_point


    def imagePt_to_realPt(self, imagePt, ratio, depth_z):
        realPt = Point32()
        realPt.x = ratio * (imagePt[0] - self.camera_info_.K[2])
        realPt.y = ratio * (imagePt[1] - self.camera_info_.K[5])
        realPt.z = depth_z

        return realPt
    '''
    def collectYaw(self,msg):
        self.yaw = msg.yaw
        
    def register(self):
        self.yaw_sub = rospy.Subscriber('/euler',compass_data,self.collectYaw)
        self.image_pub = rospy.Publisher("/Vision/image_filter",Image)
        self.image_sub = rospy.Subscriber(self.image_param, Image,self.processImage)
        self.cam_info_sub_ = rospy.Subscriber('bottomcam/camera/camera_info', CameraInfo, self.CameraInfoCB)
        rospy.loginfo("Topics registered")
    def unregister(self):
        self.yaw_sub.unregister()
        self.image_sub.unregister()
        self.image_pub.unregister()
        rospy.loginfo("Topics unregistered")
