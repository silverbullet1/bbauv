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
from Pyste.declarations import self

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
        
        if self.comms.isTesting:
            self.comms.register()
            rospy.loginfo("Starting Pegs")
        
        return 'start_complete'

class SearchYellowBoard(smach.State):
    timeout = 10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchYellow_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        
        while not self.comms.foundYellowBoard: 
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search in figure of 8? 
            rospy.sleep(rospy.Duration(0.3))   

        return 'search_complete'
    
class SearchPegs(smach.State):
    timeout = 10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchPeg_complete', 'aborted', 'killed'])
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

class MoveForward(smach.State):
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forwarding', 'forward_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):    
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
            
        if self.comms.areaRect > 10000:
            return 'forward_complete'
        
        return 'forwarding'
    
class Centering(smach.State):
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):        
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
            
        if self.comms.angleError < 0.005:
            if self.comms.foundYellowBoard:
                self.comms.timeToFindPegs = True
            return 'centering_complete'
        
        return 'centering'

class Offset(smach.State):    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['offset_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):        
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        # Move robot 
        
        # For offset 
        self.comms.sendMovement(sidemove = -0.20)
        return 'offset_complete'
        

class MovePeg(smach.State):    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['task_complete', 'move_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        if self.comms.findRedPeg: 
            # Open grabber to grab peg & wait for response
            self.comms.findRedPeg = False    
        elif not self.comms.findRedPeg:
            # Put back peg & wait for response 
            self.comms.sendMovement(forward = -2.0)     # Reverse
            
            self.comms.findRedPeg = True
            self.comms.count = self.comms.count + 1
        
            # Maybe can reverse to find yellow board again then find next beg
        
        if self.comms.count == 4:
            return 'task_complete'
        
        return 'move_complete'

def main():
    rospy.init_node('pegs_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Pegs Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCHYELLOW",
                                             'killed': 'killed'})
        
        smach.StateMachine.add("SEARCHYELLOW", SearchYellowBoard(myCom),
                                transitions={'searchYellow_complete': "CENTERING",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})        
    
        smach.StateMachine.add("SEARCHPEGS", SearchPegs(myCom),
                                transitions={'searchPeg_complete': "SEARCHYELLOW",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})       
    
        smach.StateMachine.add("MOVEFORWARD", MoveForward(myCom),
                                transitions={'forwarding': "MOVEFORWARD",
                                             'forwarding_complete': "CENTERING",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})       
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "OFFSET",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})        
 
        smach.StateMachine.add("OFFSET", Offset(myCom),
                                transitions={'offset_complete': "MOVEPEG",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})      
        
        smach.StateMachine.add("MOVEPEG", MovePeg(myCom),
                                transitions={'move_complete': "SEARCHPEG",
                                             'task_complete': "SUCCEEDED",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})      
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/PEGS')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)
