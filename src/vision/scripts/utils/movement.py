#!/usr/bin/env/python 

'''
To encompass all our movement code
'''


import rospy
import actionlib 

from bbauv_msgs.msgs import *
from bbauv_msgs.srv import *
from ctypes.wintypes import MSG
from Pyste.declarations import self

class Movement():
    def __init__(self):
        locomotionClient = actionlib.SimpleActionClient("LocomotionServer", bbauv_msgs.msg.ControllerAction)
        heading_sub = rospy.Subscriber('/euler', compass_data, self.headingCallback)
        
        depth = 1.0
        forward = 0.0
        sidemove = 0.0
        heading = 0.0
    
    def dive(depth = None):
        self.depth = depth 
        self.sendGoal()
        
    def moveForward(self, forward=0.0, heading=None, sidemove=0.0, depth=None):
        self.forward = forward
        self.heading = heading if heading else self.heading 
        self.sidemove = self.sidemove
        self.depth = depth if depth else self.depth 
        self.sendGoal()
    
    def sendGoal(self):
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = self.forward,
                                             heading_setpoint = self.heading,
                                             sidemove_setpoint = self.sidemove,
                                             depth_setpoint = self.depth)
        
        rospy.loginfo("Forward: {} heading: {} sidemove:{} depth: {}".format(forward, heading, sidemove))
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(1))
        
    def headingCallback(self, msg):
        self.heading = msg.yaw        