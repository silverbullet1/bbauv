'''
State Machine for the Flare task
'''

import roslib
import rospy
import actionlib
from rospy.timer import sleep

import smach
import smach_ros

from dynamic_reconfigure.server import Server
#from Vision.cfg import flareTaskConfig

import math
import os
import sys
import numpy as np

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import flarevision

#Starts off in disengage class
class Disengage(smach.State):
    client = None
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'complete', 'aborted'], input_keys=['complete'])
    
    def execute(self, userdata):
        #do stuff
        return 'complete'
    
#Searches for the flare
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['search_complete', 'aborted', 'mission_abort'])
    
    def execute(self, userdata):
        #do stuff
        return 'search_complete'


#Centers the robot to the flare 
class Centering(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['centering_complete', 'aborted', 'mission_abort'], output_keys=['center_pos'])
    
    def execute(self, userdata):
        #do stuff
        return 'centering_complete'

#Manuoevre the robot towards the flare 
class Manuoevre(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['manuoevre_complete', 'aborted', 'mission_abort'])
    def execute(self,userdata):
        #do stuff
        return 'manuoevre_complete'

'''
Main python thread
'''
    
def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    global fv
    
    rospy.loginfo("Flare service handled")
    
    if req.start_request:
        rospy.loginfo("Flare is Start")
        isStart = True
        isAbort = False 
        #locomotionGoal = req.start_ctrl
    if req.abort_reqest:
        rospy.loginfo("Flare abort received")
        isAbort = True
        isStart = False
        fv.unregister()
    
    #To fill accordingly
    return mission_to_visionResponse(isStart, isAbort)
    
#Global variables 
isStart = False 
isAbort = False
isEnd = False 
rosRate = None 
isTest = False
fv = None
VisionLoopCount = 0                 #Counter for number of times the image is being processed

mani_pub = None
movement_client = None
locomotionGoal = None

flare_params = {'flare_area':0, 'centering_x':0, 'centering_y':0}

#Param config callback
def flareCallback(conig, level):
        return config

if __name__ == '__main__':
    rospy.init_node('Flare', anonymous=False)
    isTestMode = rospy.get_param('~testmode', False)
    rosRate = rospy.Rate(20)
    flare = flarevision(False)
    rospy.loginfo("Flare loaded!")
    
    if isTestMode:
        isStart = True
    
    #Link to motion
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    mani_pub = rospy.Publisher("/manipulators", manipulator)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down flare")
    pass
    