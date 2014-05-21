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
toBangColour = False

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if self.comms.isTesting:
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
        
        if self.comms.rectArea > 15000:
            self.comms.sendMovement(forward=-1.5, wait=True)   #Reverse a bit
            if not self.toBangColour:
                self.comms.navigationRegister()
                self.toBangColour = True    # Now we bang the colours
            return 'centering_complete'
        
        return 'centering'

# For bump
class bangBuoy(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['banging', 'bang_to_center', 'bang_complete', 'aborted' 'killed'])
        self.comms = comms
        self.curHits = 0
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
  
        # To toggle between buoys
        if toBangColour:
            if self.curHits == self.comms.timesToBump:
                return 'bang_complete'
            rospy.loginfo("Banging again")
            self.comms.sendMovement(forward=2.0, wait=True)    #Move forward
            self.comms.sendMovement(forward=-2.0, wait=True)    #Reverse
            self.curHits = self.curHits + 1
            
            # Return to original position
            self.comms.goToPos()
            
            return 'bang_again'        
  
        else:
            # First time to bang 
            # Move forward & correct heading 
            self.comms.sendMovement(forward=1.0)
            return 'banging'
        
        if self.comms.rectArea > 15000:
            return 'bang_to_center'

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
                                            'centering_complete': "BANGBUOY",
                                            'aborted': 'aborted',
                                            'killed': 'killed'})
        
        smach.StateMachine.add("BANGBUOY", bangBuoy(myCom),
                               transitions={'banging': "BANGBUOY",
                                            'bangToCenter': "CENTERING",
                                            'bang_complete': "DISENGAGE",
                                            'aborted': 'aborted',
                                            'killed': 'killed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/RGB_BUOY')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)