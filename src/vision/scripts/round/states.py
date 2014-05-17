'''
Smach state machine for going round the mountain
'''

import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from dynamic_reconfigure.server import Server

from vision import RoundVision

# Globals 
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
            rospy.loginfo("Starting Round")
        
        return 'start_complete'
    
class Search(smach.State):
    timeout = 10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        while not self.comms.foundSomething:
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search in figure of 8? 
            rospy.sleep(rospy.Duration(0.3))
        
        return 'search_complete'
    
# Precise movements when near centroid
class Centering (smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
# Shoot forward across the rod
class ShootForward(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_forward_once', 'shoot_forward_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        # Move forward 3m
        
        if self.comms.firstCross:
            self.comms.firstCross = False 
            return 'shoot_forward_once'
        
        return 'shoot_forward_complete'
    
# After the first cross, turn back towards the structure
class TurnBackAround(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['turn_back_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        # Turn right 90 deg
        # Move forward 2 m -- TODO: Check dimensions 
        # Turn right 90 deg 
        
        return 'turn_back_complete'
    
def main():
    rospy.init_node('round_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Round Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                         'killed': 'killed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/ROUND')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)
        