'''
Smach state machine for Torpedo
'''

import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from comms import Comms
import time
from utils.utils import Utils

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
            rospy.sleep(rospy.Duration(0.8))
            self.comms.inputHeading = self.comms.curHeading
            rospy.loginfo("Starting Torpedo")

        rospy.loginfo("Heading: {}".format(self.comms.inputHeading))
        self.comms.sendMovement(depth=self.comms.defaultDepth,
                                heading=self.comms.inputHeading,
                                blocking=True)
        
        return 'start_complete'

class FollowSonar(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['sonar_complete', 'following_sonar', 'aborted', 'killed'])
        self.comms = comms
        
        self.comms.registerSonar()
        rospy.sleep(duration=0.5)
        
    def execute(self, ud):
        if self.comms.sonarDist > 2:
            self.comms.sendMovement(forward=self.comms.sonarDist,
                                    heading=self.comms.sonarBearing,
                                    timeout=0.5, blocking=False)
            return 'following_sonar'
        else:
            return 'sonar_complete'
        
class SearchCircles(smach.State):
    timeout = 1000
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchCircles_complete', 'aborted', 'killed'])
        self.comms = comms
        
        # Unregister sonar sub
#         self.comms.unregisterSonar
    
    def execute(self, ud):
        start = time.time()
        
        if not self.comms.foundCircles and self.comms.foundSomething:
            if self.comms.foundCount < 20:
                # Search pattern 
                self.comms.sendMovement(forward=0.2, heading=self.comms.curHeading+20,
                                        sidemove=-0.4, blocking=True)
                self.comms.sendMovement(forward=0.2, heading=self.comms.curHeading-20,
                                        sidemove=0.4, blocking=True)

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
    deltaYMult = 0.2
    completeRadius = 30
    lostCount = 0
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forwarding', 'forward_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):    
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        if self.lostCount > 10:
            return 'lost'
        
        if not self.comms.foundCircles:
            self.lostCount = self.lostCount + 1
                    
        if self.comms.radius > self.completeRadius:
            return 'forward_complete'
        
        if abs(self.comms.deltaY) > 0.010:
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            # Prevent surfacing
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = 2.0
                
        # Side move and keep heading
        self.comms.sendMovement(forward=self.forward_setpoint, 
                                sidemove=self.comms.deltaX * self.deltaXMult, 
                                depth=self.comms.defaultDepth, timeout=0.4, 
                                blocking=False)
        
        return 'forwarding'

# Precise movements when near centroid
class Centering (smach.State):
    deltaXMult = 3.0
    deltaYMult = 0.2
    depthCount = 0
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
            
        if self.count > 50:
            return 'centering_complete'

        if abs(self.comms.deltaY) > 0.010:
            if self.comms.radius > 50:
                self.deltaYMult = 0.05
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = 2.0
        
        # Sidemove and center
        self.comms.sendMovement(forward = 0.0,
                                sidemove = self.comms.deltaX * self.deltaXMult,
                                depth = self.comms.defaultDepth,
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
            self.comms.taskComplete()
            return 'shoot_complete'
        
        # Reset parameters
        self.comms.centroidToShoot = None
        self.comms.numShoot = self.comms.numShoot + 1
        
        return 'shoot_again'
    
def main():
    rospy.init_node('torpedo_lynnette_awesomeness', anonymous=False)
    rosRate = rospy.Rate(30)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCHCIRCLES",
                                         'killed': 'killed'})      
        
        smach.StateMachine.add("FOLLOWSONAR", FollowSonar(myCom),
                               transitions={'following_sonar': "FOLLOWSONAR",
                                           'sonar_complete': "SEARCHCIRCLES",
                                           'aborted': 'aborted',
                                           'killed': 'killed'})
    
        smach.StateMachine.add("SEARCHCIRCLES", SearchCircles(myCom),
                                transitions={'searchCircles_complete': "SEARCHCIRCLES",
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
        
