import roslib; roslib.load_manifest('vision')
import rospy
import actionlib
from sensor_msgs.msg import Image
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *

import cv2

import math
import numpy as np

class LineFollower():
    thval = 15
    areaThresh = 10000
    screen = {}
    screen['width'] = 640
    screen['height'] = 480
    
    #Defualt setpoints
    f_setpoint = 0; h_setpoint = 0; sm_setpoint = 0;
    d_setpoint = 0; r_setpoint = 0; p_setpoint = 0;

    locomotionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                    bbauv_msgs.msg.ControllerAction) 

    def __init__(self):
        self.cvbridge = CvBridge()
        self.rectData = {'detected':False}

        #Subscribe to camera
        self.imgSub = rospy.Subscriber("/bot_camera/camera/image_rect_color_opt",
                                        Image,
                                        self.cameraCallback)
        #Subscribe to compass
        self.comSub = rospy.Subscriber("/euler",
                                        compass_data,
                                        self.compassCallback)
        #Publisher for testing output image
        self.outPub = rospy.Publisher("/botcam/filterimage", Image)
    
    #ROS callback functions
    def cameraCallback(self, image):
        cvImg = self.cvbridge.imgmsg_to_cv2(image, image.encoding)
        self.detectBlackLine(cvImg)
    
    def compassCallback(self, data):
        h_setpoint = data.yaw
        r_setpoint = data.roll
        p_setpoint = data.pitch

    #Utility function to send movements throught locomotion server
    def sendMovement(f=f_setpoint, h=h_setpoint, sm=sm_setpoint, d=d_setpoint,
                     r=r_setpoint, p=p_setpoint):
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=f, heading_setpoint=h,
                                             sidemove_setpoint=sm, depth_setpoint=d,
                                             roll_setpoint=r, pitch_setpoint=p)
        locomotionClient.sendGoal(goal)

    #Main filters chain
    def detectBlackLine(self, img):
        grayImg = cv2.cvtColor(img, cv2.cv.CV_BGR2GRAY)
        grayImg = cv2.resize(grayImg,
                dsize=(self.screen['width'], self.screen['height']))

        #Thresholding and noise removal
        grayImg = cv2.GaussianBlur(grayImg, ksize=(5, 5), sigmaX=0)
        grayImg = cv2.threshold(grayImg, self.thval, 255, cv2.THRESH_BINARY_INV)[1] 
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        grayImg = cv2.erode(grayImg, erodeEl)
        grayImg = cv2.dilate(grayImg, dilateEl)
        
        #Find centroid and bouding box
        pImg = grayImg.copy()
        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        maxArea = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.areaThresh and area > maxArea:
                #Find the center using moments
                mu = cv2.moments(contour, False) 
                centroidx = mu['m10']/mu['m00']
                centroidy = mu['m01']/mu['m00']
                maxArea = area
                
                self.rectData['centroid'] = (centroidx, centroidy)
                self.rectData['rect'] = cv2.minAreaRect(contour)

        if maxArea > 0:
            self.rectData['detected'] = True;
            points = np.array(cv2.cv.BoxPoints(self.rectData['rect']))

            #Find the blackline heading
            edge1 = points[1] - points[0] 
            edge2 = points[2] - points[1]
            #Choose the vertical edge
            if cv2.norm(edge1) > cv2.norm(edge2):
                self.rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))
            else:
                self.rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))
            #Choose angle to turn if horizontal

            #Chose angle to turn if horizontal
            if self.rectData['angle'] == 90:
                if self.rectData['centroid'][0] > self.screen['width'] / 2:
                    self.rectData['angle'] = -90
            elif self.rectData['angle'] == -90:
                if self.rectData['centroid'][0] < self.screen['width'] / 2:
                    self.rectData['angle'] = 90
      
            #Testing
            out = img.copy()
            centerx = int(self.rectData['centroid'][0])
            centery = int(self.rectData['centroid'][1])
            cv2.circle(out, (centerx, centery), 5, (0, 255, 0)) 
            for i in range(4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                cv2.line(out, pt1, pt2, (0, 0, 255))
            cv2.putText(out, str(self.rectData['angle']), (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
            #For gray scale testing
            #out.shape = (out.shape[0], out.shape[1], 1)
            self.outPub.publish(self.cvbridge.cv2_to_imgmsg(out, encoding="bgr8"))
