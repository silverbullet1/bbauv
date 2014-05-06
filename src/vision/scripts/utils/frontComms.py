#!/usr/bin/env python

'''
Communication b/w ROS class and submodules for front camera
'''

import sys
import importlib

import rospy
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data

from utils import Utils
import config 

class FrontComms:
    inputHeading = 0
    curHeading = 0
    
    def __init__(self, subModule):
        mod = importlib.import_module(subModule)
        #Flag for using non-publishing to ROS when testing with images 
        self.canPublish = False
        
        #Initialize vision Filter
        self.visionFilter = mod
        
        #Get private params 
        self.imageTopic = rospy.get_param('~image', config.frontCamTopic)
        
    def register(self):
        self.camSub = rospy.Subscriber(self.imageTopic, Image, self.camCallback)
        self.compassSub = rospy.Subscriber(config.compassTopic,
                                           compass_data,
                                           self.compassCallback)
        self.outPub = rospy.Publisher(config.visionFilterTopic, Image)
        
    def unregister(self):
        self.camSub.unregister()
        self.compassSub.unregister()
        #self.outPub.unregister()
    
    def camCallback(self, rosImg):
        outImg = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
        if self.canPublish:
            self.outPub.publish(Utils.cv2rosimg(outImg))
            
    def compassCallback(self, data):
        self.curHeading = data.yaw
        