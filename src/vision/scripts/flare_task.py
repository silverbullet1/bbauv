#!/usr/bin/env python
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

import math
import os
import sys
import numpy as np

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from flare_vision import Flare

#Global variables 
isStart = False
isEnd = False
isTestMode = False                  #If test mode then don't wait for mission call  
rosRate = None 
flare = None
VisionLoopCount = 0                 #Counter for number of times the image is being processed

mani_pub = None
movement_client = None
locomotionGoal = None

flare_params = {'flare_area':0, 'centering_x':0, 'centering_y':0}

#Starts off in disengage class
class Disengage(smach.State):
    
    def __init__(self, flare_task):
        smach.State.__init__(self, outcomes=['start_complete', 'complete_outcome', 'aborted'])
        self.flare = flare_task
    
    def execute(self, userdata):
        self.flare.unregister()

        while self.flare.isAborted:
            rospy.sleep(rospy.Duration(0.2))
        
        self.flare.register()
        rospy.loginfo("Starting Flare")
        return 'start_complete'
    
#Searches for the flare
class Search(smach.State):
    timeout = 100    #5s timeout before aborting task
    def __init__(self, flare_task):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'mission_abort'])
        self.flare = flare_task
    
    def execute(self, userdata):
        #Check for abort signal
        if self.flare.isAborted:
            return 'aborted'
        
        #Check if flare found or timeout already
        timecount = 0
        while not self.flare.rectData['detected']:
            if timecount > self.timeout or rospy.is_shutdown() or self.flare.isKilled:
                self.flare.abortMission()
                return 'aborted'
            self.flare.sendMovement(forward=1.0, depth=0.4)
            rospy.sleep(rospy.Duration(0.3))
            timecount += 1
            self.flare.failedTask();
        
        return 'search_complete'

#Bash towards the flare!
class Manuoevre(smach.State):
    def __init__(self, flare_task):
        smach.State.__init__(self, outcomes=['manuoevring', 'manuoevre_complete',
                                             'aborted', 'mission_abort'])
        self.flare = flare_task
        self.deltaThresh = 0.15
        self.prevAngle = []
        self.count = 0
        
    def execute(self,userdata):
        #Check for aborted signal
        if self.flare.isAborted:
            return 'aborted'
        
        #Cannot detect already
#         if not self.flare.rectData['detected']:
#             self.count += 1
#         if self.count > 4:
#             self.flare.taskComplete()
#             return 'manuoevre_complete'
        
        #Get to the flare
        screenWidth = self.flare.screen['width']
        screenCenterX = screenWidth / 2
        deltaX = (self.flare.rectData['centroids'][0] - screenCenterX) / screenWidth
        rospy.loginfo(deltaX)
        
        #Forward if center
        #Shoot straight and aim
        if self.flare.rectData['area'] > self.flare.headOnArea and abs(deltaX) < 0.15:
            self.flare.sendMovement(forward=1.5)
            rospy.loginfo("Hitting flare")
            rospy.loginfo("Forward 1.5")
            self.flare.taskComplete()
            return 'manuoevre_complete'
        #Forward if center
        elif abs(deltaX) < 0.15:
            self.flare.sendMovement(forward=self.flare.forwardOffset)
            rospy.loginfo("Forward {}".format(self.flare.forwardOffset))
        else:
            #Sidemove if too far off center
            sidemove = math.copysign(deltaX*self.flare.deltaXMultiplier, deltaX)     #Random number
            self.flare.sendMovement(forward=self.flare.sidemoveMovementOffset, sidemove=sidemove)
            rospy.loginfo("Forward {} sidemove{}".format(self.flare.sidemoveMovementOffset,sidemove))
        return 'manuoevring'
                       
'''
Main python thread
'''
    
def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    global flare
    
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
        flare.unregister()
    
    #To fill accordingly
    return mission_to_visionResponse(isStart, isAbort)
    
#Param config callback
def flareCallback(conig, level):
    for param in flare.yellow_params:
        flare.yellow_params[param] = config['yellow_' + param]
    isTestMode = config["testing"]
    return config

#Utility function for normalising heading 
def normHeading(heading):
    if heading > 360:
        return heading - 360
    elif heading < 0:
        return heading + 360
    else:
        return heading 

if __name__ == '__main__':
    rospy.init_node("Flare", anonymous=False)
    rosRate = rospy.Rate(20)
    flare_task = Flare()
    rospy.loginfo("Flare loaded!")
    
    #Create state machine container 
    sm = smach.StateMachine(outcomes=['complete_flare', 'aborted'])
    
    #Disengage, Search, Manuoevre
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(flare_task),
                               transitions={'start_complete': "SEARCH", 
                                            'complete_outcome': 'complete_flare', 
                                            'aborted': 'aborted'})
        
        smach.StateMachine.add("SEARCH", Search(flare_task),
                               transitions={'search_complete': "MANUOEVRE", 'aborted': 'aborted', 
                                            'mission_abort': "DISENGAGE"})
    
        smach.StateMachine.add("MANUOEVRE", Manuoevre(flare_task),
                               transitions = {'manuoevring': "MANUOEVRE",
                                              'manuoevre_complete': "DISENGAGE",
                                              'aborted': 'aborted',
                                              'mission_abort': "DISENGAGE"})
    
    outcomes = sm.execute()
    
