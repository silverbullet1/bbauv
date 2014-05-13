#!/usr/bin/env/python

'''
Pegs states
'''

import roslib; roslib.load_manifest('vision')
import rospy 

import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from dynamic_reconfigure.server import Server

from vision import PegsVision

#Globals
locomotionGoal = None
isTesting = False
isKilled = False
isAborted = False

class Disengage(smach.State):
    def __init__(self, pegs):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.pegs = pegs
        
    def execute(self, userdata):
        if isKilled:
            rospy.signal_shutdown("Bye")
            return 'killed'

        while isAborted:
            rospy.sleep(rospy.Duration(0.2))

            if isTesting:
                self.pegs.register()
                rospy.loginfo("Starting Pegs")
                
            return 'start_complete'
        
class SearchRedPeg(smach.State):
    def __init__(self, pegs):
        smach.State.__init__(self, outcomes=['searchRed_complete', 'aborted', 'killed'])
        self.pegs = pegs
    
    def execute(self, ud):
        smach.State.execute(self, ud)

class Centering(smach.State):
    def __init__(self, pegs):
        smach.State.__init__(self, outcomes=['centering_complete', 'centering', 'aborted', 'killed'])
        self.pegs = pegs
    
    def execute(self, ud):
        smach.State.execute(self, ud)

class OffSet(smach.State):
    def __init__(self, pegs):
        smach.State.__init__(self, outcomes=['offset_complete', 'aborted', 'killed'])
        self.pegs = pegs
    
    def execute(self, ud):
        smach.State.execute(self, ud)

class TakeRedPeg(smach.State):
    def __init__(self, pegs):
        smach.State.__init__(self, outcomes=['takeRedPeg_complete', 'takingRedPeg', 'aborted', 'killed'])
        self.pegs = pegs
    
    def execute(self, ud):
        smach.State.execute(self, ud)
        
class PutWhitePeg(smach.State):
    def __init__(self, pegs):
        pass
    
    def execute(self, ud):
        smach.State.execute(self, ud)
    
def main():
    rospy.init_node('pegs_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Pegs Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                         'killed': 'killed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/PEGS')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)