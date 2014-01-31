import roslib; roslib.load_manifest('vision')
import rospy

import actionlib
from sensor_msgs.msg import Image
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *

import cv2

class LineFollower():
    thval = 30
    areaThresh = 9000
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
        cv2.GaussianBlur(grayImg, ksize=(5, 5), sigmaX=0)
        cv2.threshold(grayImg, thval, 255, cv2.THRESH_BINARY_INV) 
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        grayImg = cv2.erode(grayImg, erodeEl)
        grayImg = cv2.dilate(grayImg, dilateEl)
        
        #Find centroid and bouding box
        contours = cv2.findContours(grayImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
       
        #Testing
        out = grayImg.copy()
        self.outPub.publish(self.cvbridge.cv2_to_imgmsg(img, encoding="bgr8"))
