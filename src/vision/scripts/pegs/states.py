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
pegCount = 0    # We try moving total 4 times 

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'aborted', 'killed'])
        self.comms = comms
        
    def execute(self, userdata):
        while not self.comms.foundRedPeg:
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
    timeout = 120
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'timeout', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):
        start = time.time()
        
        while not self.comms.foundRedPeg:
            if (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'timeout'
            if self.comms.isKilled or self.comms.isAborted:
                return 'aborted'
        
        return 'search_complete'
    
class ForwardToPeg(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forward', 'forward_complete', 'lost', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):       
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
                
        return 'forward_complete' 
    
class Centering(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'lost', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):       
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
                
        return 'centering_complete'      
    
class Offset(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['offset', 'offset_complete', 'lost', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):       
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
                
        return 'offset_complete'   
    
class MovePeg(smach.State):
    global pegCount
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['move_complete', 'task_complete', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):       
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
        
        if self.comms.findRedPeg:
            # open the manipulator to grab
            self.comms.findRedPeg = False

        elif not self.comms.findRedPeg:
            # Close manipulator to put peg
            self.comms.findRedPeg = True
            pegCount = pegCount + 1
            # Return to (x,y) coordinate
            
        if pegCount == 4:
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
                                transitions={'start_complete': "SEARCH",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})
        
        smach.StateMachine.add("SEARCH", Search(myCom),
                                transitions={'search_complete': "SEARCH",
                                             'timeout': 'killed',
                                             'killed': 'killed'})     
        
        smach.StateMachine.add("FORWARD", ForwardToPeg(myCom),
                                transitions={'forward': "FORWARD",
                                             'forward_complete': "CENTERING",
                                             'lost': "SEARCH",
                                             'aborted': 'aborted'})             
    
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "OFFSET",
                                             'lost': "SEARCH",
                                             'aborted': 'aborted'})  
        
        smach.StateMachine.add("OFFSET", Offset(myCom),
                                transitions={'offset_complete': "MOVEPEG",
                                             'lost': "SEARCH",
                                             'aborted': 'aborted'})  
 
        smach.StateMachine.add("MOVEPEG", MovePeg(myCom),
                                transitions={'move_complete': "SEARCH",
                                             'task_complete': 'succeeded',
                                             'aborted': 'aborted'})  
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/PEGS')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)