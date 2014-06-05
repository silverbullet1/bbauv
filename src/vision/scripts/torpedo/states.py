'''
Smach state machine for torpedo
'''

import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from comms import Comms
import time

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
        
        if self.comms.isAlone:
            self.comms.register()
            rospy.loginfo("Starting Torpedo")
        
        return 'start_complete'

class SearchCircles(smach.State):
    timeout = 1000
    
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

        return 'searchCircles_complete'

class MoveForward(smach.State):
    forward_setpoint = 0.3
    deltaXMult = 5.0
    completeRadius = 30
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forwarding', 'forward_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):    
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        if not self.comms.foundCircles:
            return 'lost'
            
        if self.comms.radius > self.completeRadius:
            return 'forward_complete'
        
        # Side move and keep heading
        self.comms.sendMovement(forward=self.forward_setpoint, 
                                sidemove=self.comms.deltaX * self.deltaXMult, 
                                blocking=False)
        
        return 'forwarding'

# Precise movements when near centroid
class Centering (smach.State):
    deltaXMult = 3.0
    deltaYMult = 2.0
    count = 0
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        rospy.loginfo("Delta X: {}".format(self.comms.deltaX))
        
        if abs(self.comms.deltaX) < 0.005 and abs(self.comms.deltaY) < 0.005:
            self.count = self.count + 1
            rospy.loginfo("Count: {}".format(self.count))
            
        if self.count > 1000:
            return 'centering_complete'

        if self.depthCount < 10:
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            # Make sure it doesnt surface
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = 0.1
            self.comms.sendMovement(depth=self.comms.defaultDepth, blocking=True)
            self.depthCount = self.depthCount + 1
            rospy.loginfo("Depth corrected")
        
        # Sidemove and center
        self.comms.sendMovement(forward = 0.0,
                                sidemove = self.comms.deltaX * self.deltaXMult,
                                blocking = False)
        return 'centering'        
        
# Shoot torpedo forward
class ShootTorpedo(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_again', 'shoot_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        # Shoot once more 
        if self.comms.numShoot == 0:
            self.comms.shootTopTorpedo()
        elif self.comms.numShoot == 1:
            self.comms.shootBotTorpedo()
        else:
            return 'shoot_complete'
        
        # Reset parameters
        self.comms.centroidToShoot = None
        self.comms.numShoot = self.comms.numShoot + 1
        
        return 'shoot_again'
    
def main():
    rospy.init_node('torpedo_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCHCIRCLES",
                                         'killed': 'killed'})      
    
        smach.StateMachine.add("SEARCHCIRCLES", SearchCircles(myCom),
                                transitions={'searchCircles_complete': "MOVEFORWARD",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})      
        
        smach.StateMachine.add("MOVEFORWARD", MoveForward(myCom),
                                transitions={'forwarding': "MOVEFORWARD",
                                             'forward_complete': "CENTERING",
                                             'lost': "SEARCHCIRCLES",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})       
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "SHOOTING",
                                             'lost': "SEARCHCIRCLES",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})  
        
        smach.StateMachine.add("SHOOTING", ShootTorpedo(myCom),
                                transitions={'shoot_again': "SEARCHCIRCLES",
                                             'shoot_complete': 'succeeded',
                                             'aborted': 'aborted',
                                             'killed': 'killed'})     
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/TORPEDO')
    introServer.start()
    
    sm.execute()
        