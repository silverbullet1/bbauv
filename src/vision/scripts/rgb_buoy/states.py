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
isTesting = False
isKilled = False
isAborted = False

class Disengage(smach.State):
    def __init__(self, rgb_buoy):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.rgb = rgb_buoy
    
    def execute(self, userdata):
        if isKilled:
            rospy.signal_shutdown("Bye")
            return 'killed'
            
        while isAborted:
            rospy.sleep(rospy.Duration(0.2))
        
        if isTesting:
            self.rgb.register()
            rospy.loginfo("Starting RGB")
        
        return 'start_complete'
    
class Search(smach.State):
    def __init__(self, rgb_buoy):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'killed'])
        self.rgb = rgb_buoy
    
    def execute(self, ud):
        smach.State.execute(self, ud)

#When lights same colour 
class ForwardToCylinder(smach.State):
    def __init__(self, rgb_buoy):
        smach.State.__init__(self, outcomes=['forward', 'forward_complete', 'aborted', 'killed'])
        self.rgb = rgb_buoy
    
    def execute(self, ud):
        smach.State.execute(self, ud)
        
#Precise movements when near cylinder 
class Centering (smach.State):
    def __init__(self, rgb_buoy):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted' 'killed'])
        self.rgb = rgb_buoy
    
    def execute(self, userdata):
        smach.state.execute(self, ud)

#Handle mission services
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

def main():
    rospy.init_node('rgb_buoy_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()
    rgb_buoy = RgbBuoyVision()
    rospy.loginfo("RGB Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(rgb_buoy),
                                transitions={'start_complete': "SEARCH",
                                         'killed': 'killed'})
        
        smach.StateMachine.add("SEARCH", Search(rgb_buoy),
                               transitions={'search_complete': "FORWARDTOCYLINDER",
                                            'aborted': 'aborted', 
                                            'killed': 'killed'})
        
        smach.StateMachine.add("FORWARDTOCYLINDER", ForwardToCylinder(rgb_buoy),
                               transitions={'forward': "FORWARDTOCYLINDER",
                                            'forward_complete': "CENTERING", 
                                            'aborted': 'aborted',
                                            'killed': 'killed'})
        
        smach.StateMachine.add("CENTERING", Centering(rgb_buoy),
                               transitions={'centering': "CENTERING",
                                            'centering_complete': "DISENGAGE",
                                            'aborted': 'aborted',
                                            'killed': 'killed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/RGB_BUOY')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)