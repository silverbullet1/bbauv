#!/usr/bin/env python

'''
Communication b/w ROS class and submodules for front camera
'''
import rospy
import actionlib
import signal

from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data, ControllerAction, ControllerGoal
from bbauv_msgs.srv import set_controller

from utils.utils import Utils
import utils.config as config

class FrontComms:
    
    def __init__(self, visionFilter):
        signal.signal(signal.SIGINT, self.userQuit)
        
        #Default parameters
        self.inputHeading = 0
        self.curHeading = 0
        self.gotHeading = False 
        self.retVal = 0
        self.defaultDepth = 0.2
        
        # Flags 
        self.canPublish = False    #Flag for using non-publishing to ROS when testing with images 
        self.isAborted = False
        self.isKilled = False
        
        #Initialize vision Filter
        self.visionFilter = visionFilter
        
        #Get private params 
        self.imageTopic = rospy.get_param('~image', config.frontCamTopic)
        self.isAlone = rospy.get_param('~alone', True)
        
        #Locomotion servers 
        self.motionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                         ControllerAction)
        try:
            rospy.loginfo("Waiting for Locomotion Server...")
            self.motionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.loginfo("Locomotion server timeout!")
            self.isKilled = True
            
        setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        setServer(forward = True, sidemove = True, heading = True, depth = True,
                  pitch = True, roll = True, topside = False, navigation = False)
        
        #Run if in alone mode 
        if self.isAlone:
            self.isAborted = False
            self.canPublish = True       
        
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
        if not self.gotHeading:
            self.curHeading = data.yaw
            self.gotHeading = True
    
    def userQuit(self, signal, frame):
        self.unregister()
        self.isAborted = True
        self.isKilled = True
        rospy.signal_shutdown("Task manually killed")
        
    def sendMovement(self, forward=0.0, sidemove=0.0,
                     heading=None, depth=None,
                     timeout=0.2, blocking=False):
        
        depth = depth if depth else self.defaultDepth
        heading = heading if heading else self.curHeading
        
        goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=heading,
                              sidemove_setpoint=sidemove, depth_setpoint=depth)
        self.motionClient.send_goal(goal)
        rospy.loginfo("Moving.. f: %lf, sm: %lf, h: %lf, d: %lf", 
                      forward, sidemove, heading, depth)

        if blocking:
            self.motionClient.wait_for_result()
        else:
            self.motionClient.wait_for_result(timeout=rospy.Duration(timeout))
        
        
        