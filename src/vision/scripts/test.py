#!/usr/bin/env python
import rospy
import os
import roslib
from bbauv_msgs.msg import * 
from bbauv_msgs.srv import * 
from sensor_msgs.msg import Image
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import math
import time


#Global constant


class muslimVision():

    def __init__(self):
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/front_camera/camera/image_raw_jin", Image, self.imgCallback)
        self.img_pub = rospy.Publisher("/Vision/filter_mask", Image)
        self.img_pub2 = rospy.Publisher("/Vision/enhanced_image", Image)
        self.screen_w= 640
        self.screen_h= 480
        self.center = {'x': 160, 'y': 120}
        self.red_min = np.array([90,0,0], dtype='uint8')
        self.red_max = np.array([179, 255,130], dtype='uint8')
        self.green_min = np.array([20,0,0], dtype='uint8')
        self.green_max = np.array([83,255,255], dtype='uint8')
        self.img_data = {'area': 0, 'centroid':0, 'distance':0, 'angle':0, 'width':0 }
        self.constDist = 1.0
        self.actualWidth = 0.05
        self.incline = ""
        self.corners = [] #top-left, top-right, bot-left, bot-right
        #Length of pvc = 1.8m
        #Pole = 0.9m
        self.vehicle_width = 0.6 # In meter

        self.depth = 0
        self.heading = 0
        self.isKilled = False

    def imgCallback(self, data):
        rospy.loginfo("Inside image_callback")
        try:
            tmp = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_img = np.array(tmp, dtype=np.uint8)
            #cv2.namedWindow('video_stream')
            #cv2.imshow('video_stream', cv_img)
            rospy.loginfo("Show image")
            ros_img = self.findPole(cv_img, True)
            output = self.cv2ros(ros_img[0])
            self.img_pub.publish(output)
            rospy.loginfo("Area: " + str(self.img_data['area']))
            f = open("area.txt", "a")
            with f:
                f.write(str(self.img_data['area']) + "\n")

            cent_x = self.img_data['centroid'][0]
            cent_y = self.img_data['centroid'][1]
            rospy.loginfo("Centroid X: " + str(cent_x) + " Y: " + str(cent_y))
            rospy.loginfo("Angle: " + str(self.img_data['angle']))
            time.sleep(0.05)
            
            
        except CvBridgeError as e:
                rospy.logerr(e)
    
    def cv2ros(self, img):
        try:
            return self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def findPole(self, img, isTest):
        hsv_img = self.toHsv(img) #Converted to HSV & crop for current usage
        enhanced = self.enhance(hsv_img)
        (mask_red, mask_green) = self.inRange(enhanced)
        mask_red_bgr = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)
        mask_green_bgr = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)
        cnt_red = self.createBounding(mask_red, mask_red_bgr)
        cnt_green = self.createBounding(mask_green, mask_green_bgr)
        (mom_red, area_red, perimeter_red) = self.getContourInfo(cnt_red)
        (mom_green, area_green, perimeter_green) = self.getContourInfo(cnt_green)
        self.drawCentroid(mom_red, mask_red_bgr)
        self.drawCentroid(mom_green, mask_green_bgr)
        self.img_data['angle'] = self.getAngle(cnt_red)
        self.drawCenter(mask_red_bgr)
        self.img_pub2.publish(self.cv2ros(cv2.cvtColor(enhanced, cv2.COLOR_HSV2BGR)))
        '''
        if isTest:
            cv2.namedWindow('ori', cv2.WINDOW_NORMAL)
            cv2.namedWindow('mask_red', cv2.WINDOW_NORMAL)
            cv2.namedWindow('mask_green', cv2.WINDOW_NORMAL)
            cv2.imshow('ori', img)
            cv2.imshow('mask_red', mask_red_bgr)
            cv2.imshow('mask_green', mask_green)
            k = cv2.waitKey(0)
            if k == 27:
                cv2.destroyAllWindows()
        '''
        return (mask_red_bgr, mask_green)


    def approxDistance(self):
        Z = self.constDist 
        W = self.actualWidth
        w = self.width
        f = w*Z/W
        self.sendMovement(forward=0.5, sidemove=0)
        time.sleep(2.0)
        new_w = self.width
        return f/(new_w)*W

    def toHsv(self,img, lowH=0, highH=1, lowW=0, highW=1):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv = np.array(hsv, dtype=np.uint8)
        hsv = hsv[self.screen_h*lowH:self.screen_h*highH,self.screen_w*lowW:self.screen_w*highW,:]
        return hsv

    def normalize(self, img):
        channel = cv2.split(img)
        cv2.normalize(channel[1], channel[1], 0, 255, cv2.NORM_MINMAX)
        channel[1] = cv2.pow(channel[1], 4)
        cv2.normalize(channel[2], channel[2], 0, 255, cv2.NORM_MINMAX)
        return cv2.merge(channel, img)

    def enhance(self, img):
        img = self.normalize(img)
        gauss = cv2.GaussianBlur(img, (3,3), 15)
        sum =  cv2.addWeighted(img, 1.1, gauss, -0.1, 0)
        return cv2.medianBlur(sum, 9)

    def inRange(self, img):
        mask_red = self.morph(cv2.inRange(img, self.red_min, self.red_max))
        mask_green = self.morph(cv2.inRange(img, self.green_min, self.green_max))
        kern_green = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        kern_red = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        mask_green = cv2.erode(mask_green, kern_green, iterations = 2)
        mask_red = cv2.dilate(mask_red, kern_red, iterations = 2)
        return (mask_red, mask_green)

    def morph(self, img):
        kern = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)
    

    def getCorner(self,box):
        x = [i[0] for i in box]
        y = [i[1] for i in box]
        top_left = (min(x), max(y))
        top_right = (max(x), max(y))
        bot_right = (max(x), min(y))
        bot_left = (min(x), min(y))
        return [top_left, top_right, bot_left, bot_right]
        
    def createBounding(self, img, draw):
        contours, hira = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        area = 0 
        for i in contours:
            curr_area = cv2.contourArea(i)
            if curr_area > area:
                area = curr_area
                cnt = i
        self.img_data['area'] = area
        rect = cv2.minAreaRect(cnt)
        box = cv2.cv.BoxPoints(rect)
        self.corner = getCorner(box)
        box = np.int0(box)
        cv2.drawContours(draw, [box], -1, (0,0,255), 1)
        return contours[0]
 
    def getContourInfo(self,cnt):
        moment = cv2.moments(cnt)
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        return (moment, area, perimeter)

    def drawCenter(self,draw):
        cv2.rectangle(draw, (self.center['x'] - 5, self.center['y'] + 5), (self.center['x'] + 5, self.center['y'] - 5), (0,255,0), 1)

    def drawCentroid(self,mom, draw):
        centroid_x = int((mom['m10']+0.0001)/(mom['m00']+0.0001))
        centroid_y = int((mom['m01']+0.0001)/(mom['m00']+0.0001))
        self.img_data['centroid'] = (centroid_x, centroid_y)
        cv2.circle(draw, (centroid_x, centroid_y), 4, (0,0,255), -1)
        return (centroid_x, centroid_y)



    def normalizeAng(self, ang, str):
        if str == "left":
            return 180 - ang
        else:
            return ang
        
    def getAngle(self,cnt):
        x =[i[0][0] for i in cnt]
        y =[i[0][1] for i in cnt]
        top_left = (min(x), max(y))
        top_right = (max(x), max(y))
        self.width = top_right[0] - top_left[0]
        bot_right = (max(x), min(y))
        bot_left = (min(x), min(y))
        hypo = math.sqrt(math.pow(bot_right[0] - top_left[0], 2) + math.pow(top_left[1]-bot_right[1], 2))
        hypo2 = math.sqrt(math.pow(top_right[0] - bot_left[0], 2) + math.pow(top_right[1]-bot_left[1], 2))
        if hypo > hypo2:
            ang = np.rad2deg(math.asin((top_left[1]-bot_right[1])/hypo))
            return normalizeAng(ang, "left")
        else:
            ang =  np.rad2deg(math.asin((top_right[1]-bot_left[1])/hypo2))
            return self.normalizeAng(ang, "right")
      


   #ROS stuff 
    def initAll(self):
    #Initialise Locomotion Client 
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except rospy.ServiceException:
            rospy.logerr("Error running Locmotion Client")
    #Initialise Controller Service:
        try:
            self.controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
            self.controllerSettings.wait_for_service()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to Controller")

    def compass_callback(self,data):
        self.heading = data.yaw

    def depth_callback(self,data):
        self.depth = data.depth 

    def sendMovement(self,forward=0.0, sidemove=0.0, turn=None, depth=1.0, absolute=False, wait=True):
        if turn is None:
            turn = self.heading
            goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        else:
            if absolute:
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
            else:
                turn = (turn+self.heading)%360 
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        rospy.loginfo("Turn received: " + str(turn))
        self.locomotionClient.send_goal(goal)
        if wait:
            self.locomotionClient.wait_for_result()
        else:
            self.locomotionClient.wait_for_result(timeout=rospy.Duration(2.0))

def main():
    isTest = rospy.get_param("~test", True)
    dir = os.getcwd()
    test_img = cv2.imread('test.jpg')
    test = muslimVision()
    #test.findPole(test_img, True)
    

if __name__ == "__main__":
    rospy.init_node("round_vision")
    main()
    rospy.spin()

