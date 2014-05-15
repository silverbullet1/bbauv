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
        smach.State.__init__(self, outcomes=['start_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        while not self.comms.foundSomething:
            if self.comms.isKilled:
                rospy.signal_shutdown("Bye")
                return 'killed'
            if self.comms.isAborted:
                rospy.signal_shutdown("User aborted")
                return 'aborted'
            
            rospy.sleep(rospy.Duration(0.3))
    
        self.comms.register()
        if self.comms.isAlone:
            self.comms.inputHeading = self.comms.curHeading
                
        return 'start_complete'
    
class Search(smach.State):
    timeout = 100
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'timeout', 'aborted'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        
        while not self.comms.foundSomething:
            if (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'timeout'
            if self.comms.isKilled or self.comms.isAborted:
                return 'aborted'
        
        return 'search_complete'
    
# Center first then shoot 
class Centering (smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted', 'lost'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
        
        if not self.comms.foundSomething:
            return 'lost'
        
        return 'centering_complete'
        
# Shoot forward across the rod
class ShootForward(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_forward_once', 'shoot_forward_complete', 'aborted '])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
        
        # Move forward 3m
        
        if self.comms.firstCross:
            self.comms.firstCross = False 
            return 'shoot_forward_once'
        
        return 'shoot_forward_complete'
    
# After the first cross, turn back towards the structure
class TurnAround(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['turn_complete', 'aborted'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
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
                                             'aborted': 'aborted',
                                             'killed': 'killed'})
        
        smach.StateMachine.add("SEARCH", Search(myCom),
                                transitions={'search_complete': "SEARCH",
                                             'timeout': 'killed',
                                             'killed': 'killed'}) 
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "SHOOTFORWARD",
                                             'lost': "SEARCH",
                                             'aborted': 'aborted'})  
        
        smach.StateMachine.add("SHOOTFORWARD", ShootForward(myCom),
                                transitions={'shoot_forward_once': "TURNAROUND",
                                             'shoot_forward_complete': "SUCCEEDED",
                                             'aborted': 'aborted'})  
        
        smach.StateMachine.add("TURNAROUND", TurnAround(myCom),
                                transitions={'turn_complete': "SEARCH",
                                             'aborted': 'aborted'})         
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/ROUND')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)
        