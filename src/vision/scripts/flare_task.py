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
isAbort = False
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
    client = None   
    def __init__(self, flare_task):
        smach.State.__init__(self, outcomes=['start_complete', 'complete_outcome', 'aborted'])
        self.flare = flare_task
    
    def execute(self, userdata):
            self.flare.unregister()
            while not rospy.is_shutdown():
                if isEnd or self.flare.isAborted:
                    return 'aborted'
                if isStart:
                    flare.register()
                    rospy.info("Starting Flare")
                    return 'start_complete'
                rospy.sleep(rospy.Duration(0.1))
        
            #Wait for mission to start call: How? :( 
            
            self.flare.register()
            return 'start_complete'
    
#Searches for the flare
class Search(smach.State):
    timeout = 50    #5s timeout before aborting task
    def __init__(self, flare_task):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'mission_abort'])
        self.flare = flare_task
    
    def execute(self, userdata):
        #Check for abort signal
        if self.flare.isAborted:
            return 'aborted'
        
        while not rospy.is_shutdown():
            if isAbort:
                rospy.loginfo("Flare aborted by Mission Planner")
                return "mission_abort"
        
        #Check if flare found or timeout already
        while not self.flare.rectData['detected']:
            if timecount > self.timeout or rospy.is_shutdown():
                self.flare.abortMission()
                return 'aborted'
            rospy.sleep(rospy.Duration(0.1))
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
        
    def execute(self,userdata):
        #Check for aborted signal
        if self.flare.isAborted:
            return 'aborted'
        
        #Cannot detect already
        if not self.flare.rectData['detected']:
            self.flare.taskComplete()
            return 'manuoevre_complete'
        
        #Get to the flare
        screenWidth = self.flare.screen['width']
        screenCenterX = screenWidth / 2
        deltaX = (rectData['centroids'][0] - screenCenterX) / screenWidth
        
        #Forward if center
        if abs(deltaX) < 0.3:
            self.flare.sendMovement(forward=0.5)
        else:
            #Sidemove if too far off center
            sidemove = deltaX * 30.0        #Random number
            self.flare.sendMovement(forward=0.2, sidemove=sidemove)
        rospy.loginfo("Forward {} sidemove{}".format(forward,sidemove))
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
    isTestMode = config["testmode"]
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
    isTestMode = rospy.get_param("~testmode", False)
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
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down flare")
    pass
    
