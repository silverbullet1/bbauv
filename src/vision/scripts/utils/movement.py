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
    def __init__(self, depth = None, forward = None, sidemove = None, heading = None):
        locomotionClient = actionlib.SimpleActionClient("LocomotionServer", bbauv_msgs.msg.ControllerAction)
        heading_sub = rospy.Subscriber('/euler', compass_data, self.headingCallback)
        
        depth = depth if depth else 1.0
        forward = forward if forward else 0.0
        sidemove = sidemove if sidemove else 0.0
        heading = heading if heading else 0.0
    
    def dive(depth = None, wait=None):
        self.depth = depth 
        self.sendGoal(wait)
        
    def moveForward(self, forward=0.0, heading=None, sidemove=0.0, depth=None, wait=1.0):
        self.forward = forward
        self.heading = heading if heading else self.heading 
        self.sidemove = self.sidemove
        self.depth = depth if depth else self.depth 
        self.sendGoal(wait)
    
    def sendGoal(self, wait):
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = self.forward,
                                             heading_setpoint = self.heading,
                                             sidemove_setpoint = self.sidemove,
                                             depth_setpoint = self.depth)
        
        rospy.loginfo("Forward: {} heading: {} sidemove:{} depth: {}".format(forward, heading, sidemove))
        self.locomotionClient.send_goal(goal)
        
        if wait:
            self.locomotionClient.wait_for_result(rospy.Duration(wait))
        else: 
            self.locomotionClient.wait_for_result()
        
    def headingCallback(self, msg):
        self.heading = msg.yaw        