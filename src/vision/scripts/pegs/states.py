#/usr/bin/env/python

'''
Pegs states
'''

import roslib; roslib.load_manifest('vision')
import rospy 

import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgsmsgs.srv import *
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
    
def handle_srv(req):
    global isStart
    global isAborted
    global locomotionGoal
    global rgb_buoy
    
    rospy.loginfo("RGB Service handled")
    
    if req.start_request:
        rospy.loginfo("RGB starting")
        isStart = True
        isAborted = False
    
    if req.abort_request:
        rospy.loginfo("Flare abort received")
        isAbort=True
        isStart = False
        Comms.unregister()
        
    return mission_to_visionResponse(isStart, isAborted)    