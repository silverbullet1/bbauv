'''
Smach state machine for torpedo
'''

import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from front_commons.frontComms import FrontComms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from dynamic_reconfigure.server import Server

from vision import TorpedoVision

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
    
class SearchGreenBoard(smach.State):
    timeout = 10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchGreen_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        
        while not self.comms.foundGreenBoard: 
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search in figure of 8? 
            rospy.sleep(rospy.Duration(0.3))   

        return 'search_complete'

class SearchCircles(smach.State):
    timeout = 10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchCircles_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        
        while not self.comms.foundCircles: 
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
        
        if self.comms.deltaX < 0.005:
            if self.comms.foundGreenBoard:
                self.comms.navigationRegister()
                self.comms.timeToFindPegs = True
            return 'centering_complete'
        
        return 'centering'        
        
# Shoot torpedo forward
class ShootTorpedo(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_again', 'shoot_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        # Shoot once more 
        if self.comms.numShoot is not 2:
            # Move back to centering of green board 
            self.comms.gotoPos()
            return 'shoot_again'
        
        return 'shoot_forward_complete'
    
    
def main():
    rospy.init_node('torpedo_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                         'killed': 'killed'})
    
        smach.StateMachine.add("SEARCHGREEN", SearchGreenBoard(myCom),
                                transitions={'searchGreen_complete': "CENTERING",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})        
    
        smach.StateMachine.add("SEARCHCIRCLES", SearchCircles(myCom),
                                transitions={'searchCircles_complete': "SEARCHYELLOW",
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
        
        smach.StateMachine.add("SHOOTING", ShootTorpedo(myCom),
                                transitions={'shoot_again': "SEARCHCIRCLES",
                                             'shoot_complete': "SUCCEEDED",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})     
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/TORPEDO')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)
        