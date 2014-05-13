#!/usr/bin/env python

'''
Buoy states
'''

import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from vision import RgbBuoyVision

from dynamic_reconfigure.server import Server

#Globals
locomotionGoal = None

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if isTesting:
            self.comms.register()
            rospy.loginfo("Starting RGB")
        
        return 'start_complete'
    
class Search(smach.State):
    timeout = 10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        while not self.comms.foundBuoy:
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search in figure of 8? 
            rospy.sleep(rospy.Duration(0.3))
        
        return 'search_complete'
        
# Precise movements when near buoy 
class Centering (smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

# For bump
class bangBuoy(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['bang_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
  
        # Move forward
        return 'bang_complete'

# For subsequent bumps
class bangColour(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['bang_colour_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
  
        # Move forward    

def main():
    rospy.init_node('rgb_buoy_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("RGB Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                         'killed': 'killed'})
        
        smach.StateMachine.add("SEARCH", Search(myCom),
                               transitions={'search_complete': "WAITFORCOLOUR",
                                            'aborted': 'aborted', 
                                            'killed': 'killed'})
    
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                               transitions={'centering': "CENTERING",
                                            'centering_complete': "DISENGAGE",
                                            'aborted': 'aborted',
                                            'killed': 'killed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/RGB_BUOY')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)