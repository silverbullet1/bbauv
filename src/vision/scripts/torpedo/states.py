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

from vision_old import TorpedoVision
from front_commons.frontCommsVision import FrontCommsVision as vision

# Globals 
locomotionGoal = None 

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
        self.comms.state = "DISENGAGE"
    
    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if self.comms.isAlone:
            self.comms.register()
            self.comms.inputHeading = self.comms.curHeading
            self.comms.sonarBearing = self.comms.curHeading
            rospy.loginfo("Starting Torpedo")
            rospy.sleep(rospy.Duration(0.8))

        self.comms.sendMovement(depth=self.comms.defaultDepth,
                                heading=self.comms.inputHeading,
                                blocking=True)
        
        return 'start_complete'

class FollowSonar(smach.State):
    sonarDist = 2.0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['sonar_complete', 'following_sonar', 'aborted', 'killed'])
        self.comms = comms        
        self.comms.registerSonar()
        rospy.sleep(duration=0.8)
        
    def execute(self, ud):
        self.comms.state = "SONARSONAR"

        if self.comms.sonarDist > self.sonarDist:
            rospy.loginfo("Sonar dist: {}, sonar bearing: {}".format(self.comms.sonarDist,
                self.comms.sonarBearing))
            self.curHeading = self.comms.sonarBearing
            self.comms.sendMovement(forward=self.comms.sonarDist,
                                    heading=self.comms.sonarBearing,
                                    timeout=0.5, blocking=False)
            return 'following_sonar'
        else:
            return 'sonar_complete'

# Align with the center of the green board 
class AlignBoard(smach.State):
    timeout = 300

    forward_setpoint = 0.3
    halfCompleteArea = 45000
    completeArea = 70000

    deltaXMult = 4.5
    deltaYMult = 0.4

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['alignBoard_complete', 'aligning_board', 'aborted', 'killed'])
        self.comms = comms

        self.comms.state = "BOARDBOARD"

    def execute(self, ud):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

        if self.comms.boardArea > self.completeArea:
            return 'alignBoard_complete'

        if abs(self.comms.boardDeltaY) > 0.010:
            if self.comms.boardArea > self.halfCompleteArea:
                self.deltaYMult = 0.2
                self.deltaXMult = 3.0

            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.boardDeltaY*self.deltaYMult
            # Prevent surfacing
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission

        if self.comms.boardArea > 40000:
            self.forward_setpoint = 0.2
                
        # Side move and keep heading
        self.comms.sendMovement(forward=self.forward_setpoint, 
                                sidemove=self.comms.boardDeltaX * self.deltaXMult, 
                                depth=self.comms.defaultDepth, timeout=0.4, 
                                blocking=False)

        return 'aligning_board'

class SearchCircles(smach.State):
    timeout = 300
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchCircles_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
        
        self.comms.state = "SEARCHING"
    
    def execute(self, ud):
        start = time.time()
        
        while not self.comms.foundCircles or not self.comms.foundSomething: 
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 

            # if not self.comms.foundCircles and self.comms.foundSomething:
            #     if self.comms.foundCount < 50:
            #         # Search pattern 
            #         self.comms.sendMovement(forward=0.2,
            #                                 sidemove=-0.4, timeout=0.5, blocking=False)
            #         self.comms.sendMovement(forward=0.2,
            #                                 sidemove=0.4, timeout=0.5, blocking=False)
            #     else:
            #         return 'lost'

            self.comms.sendMovement(forward=0.2, timeout=0.6, blocking=False)

            # Search in figure of 8? 
            rospy.sleep(rospy.Duration(0.3))   

        return 'searchCircles_complete'

# Just move forward 
class MoveForward(smach.State):
    forward_setpoint = 0.4
    halfCompleteRadius = 40
    completeRadius = 70
    lostCount = 0

    deltaXMult = 4.0
    deltaYMult = 0.3

    deltaXMult2 = 3.0
    deltaYMult2 = 0.10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forwarding', 'forward_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
        self.comms.state = "MOVING"

    def execute(self, ud):    
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted' 
        
        if self.lostCount > 40:
            return 'lost'
        
        if not self.comms.foundCircles:
            self.lostCount = self.lostCount + 1
                    
        if self.comms.radius > self.completeRadius:
            return 'forward_complete'
        
        if abs(self.comms.deltaY) > 0.010:
            if self.comms.radius > self.halfCompleteRadius:
                self.deltaYMult = self.deltaYMult2
                self.deltaXMult = self.deltaXMult2
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            # Prevent surfacing
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission
                
        # Side move and keep heading
        self.comms.sendMovement(forward=self.forward_setpoint, 
                                sidemove=self.comms.deltaX * self.deltaXMult, 
                                depth=self.comms.defaultDepth, timeout=0.4, 
                                blocking=False)
        
        return 'forwarding'

# Precise movements when near centroid
class Centering (smach.State):
    deltaXMult = 3.0
    deltaYMult = 0.05
    depthCount = 0
    count = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms

        self.comms.state = "CENTERING"
    
    def execute(self, userdata):
        self.comms.isCenteringState = True
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
                
        # if abs(self.comms.deltaX) < 0.005 and abs(self.comms.deltaY) < 0.005:
        #     self.count = self.count + 1
        #     rospy.loginfo("Count: {}".format(self.count))
            
        # if self.count > 50:
        #     return 'centering_complete'

        if abs(self.comms.deltaX) < 0.010 and abs(self.comms.deltaY) < 0.010:
            self.comms.sendMovement(forward=0.30, sidemove=0.0, depth=self.comms.defaultDepth, blocking=True)
            if not self.comms.foundSomething:
                return 'centering_complete'
            else:
                return 'centering'

        # if self.comms.radius > 70 and not self.comms.foundSomething:
        #     self.comms.lockedCentroid = self.comms.centroidToShoot
        #     rospy.loginfo("MoveForward")
        #     self.comms.sendMovement(forward = 0.20,
        #                     sidemove = 0.0, 
        #                     depth = self.comms.defaultDepth,
        #                     blocking = True)
        #     return 'centering_complete'

        if abs(self.comms.deltaY) > 0.010:
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission
        
        sidemove_setpoint = 0.0
        if abs(self.comms.deltaX) > 0.010:
            sidemove_setpoint = self.comms.deltaX * self.deltaXMult

        # Sidemove and center
        self.comms.sendMovement(forward = 0.10,
                                sidemove = sidemove_setpoint,
                                depth = self.comms.defaultDepth,
                                blocking = False)
        return 'centering'        
        
# Go to the point 
class LockedPoint(smach.State):
    deltaXLockMult = 4.0
    deltaYLockMult = 0.3
    correctionDone = False 

    deltaXMult = 2.0
    deltaYMult = 0.1

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['locking', 'locking_complete', 'aborted', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        self.comms.state = "LOCKING"

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

        # One time move towards the centroid 
        if not self.correctionDone:
            deltaX = float((self.comms.lockedCentroid[0] - vision.screen['width']/2)*1.0/
                                        vision.screen['width'])
            deltaY = float((self.comms.lockedCentroid[1] - vision.screen['height']/2)*1.0/
                                vision.screen['height'])
            rospy.loginfo(deltaX)
            rospy.loginfo(deltaY)

            self.comms.defaultDepth = self.comms.defaultDepth * self.deltaYLockMult
            self.comms.sendMovement(forward = 0.0,
                                sidemove = self.comms.deltaX * self.deltaXLockMult,
                                depth = self.comms.defaultDepth,
                                blocking = True)
            rospy.sleep(rospy.Duration(0.3))
            self.correctionDone = True
        else: 
            if self.comms.radius > 90:
                return 'locking_complete'
            else:
                self.comms.defaultDepth = self.comms.deltaY * self.deltaYMult
                if self.comms.defaultDepth < 0.1:
                    self.comms.defaultDepth = self.comms.depthFromMission
            
                self.comms.sendMovement(forward = 0.1,
                                        sidemove = self.comms.deltaX * self.deltaXMult,
                                        depth = self.comms.defaultDepth,
                                        blocking = False)


        return 'locking'

# Shoot torpedo forward
class ShootTorpedo(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_again', 'shoot_complete', 'aborted', 'killed'])
        self.comms = comms

        self.comms.state = "SHOOT TO KILL"
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        if self.comms.numShoot == 0:
            rospy.loginfo("SHOOT")

            self.comms.sendMovement(forward = -2.0,
                                    sidemove = 0.0,
                                    depth = self.comms.defaultDepth,
                                    blocking = True)

            self.comms.shootTopTorpedo()
        elif self.comms.numShoot == 1:
            rospy.loginfo("SHOOT 2")
            self.comms.shootBotTorpedo()
        
        # Reset parameters
        self.comms.centroidToShoot = None
        self.comms.numShoot = self.comms.numShoot + 1

        if self.comms.numShoot == 2:
            self.comms.taskComplete()
            return 'shoot_complete'
        
        return 'shoot_again'

def main():
    rospy.init_node('torpedo_lynnette_awesomeness', anonymous=False)
    rosRate = rospy.Rate(30)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'failed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "FOLLOWSONAR",
                                         'killed': 'failed'})      
        
        smach.StateMachine.add("FOLLOWSONAR", FollowSonar(myCom),
                               transitions={'following_sonar': "FOLLOWSONAR",
                                           'sonar_complete': "FOLLOWSONAR",
                                           'aborted': 'aborted',
                                           'killed': 'failed'})

        smach.StateMachine.add("ALIGNBOARD", AlignBoard(myCom),
                                transitions={'aligning_board': "ALIGNBOARD",
                                            'alignBoard_complete': "MOVEFORWARD",
                                            'aborted': 'aborted', 
                                            'killed': 'failed'
                                })
    
        smach.StateMachine.add("SEARCHCIRCLES", SearchCircles(myCom),
                                transitions={'searchCircles_complete': "SEARCHCIRCLES",
                                             'lost': 'failed',
                                             'aborted': 'aborted',
                                             'killed': 'failed'})      
        
        smach.StateMachine.add("MOVEFORWARD", MoveForward(myCom),
                                transitions={'forwarding': "MOVEFORWARD",
                                             'forward_complete': "CENTERING",
                                             'lost': "SEARCHCIRCLES",
                                             'aborted': 'aborted',
                                             'killed': 'failed'})       
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "SHOOTING",
                                             'lost': "SEARCHCIRCLES",
                                             'aborted': 'aborted',
                                             'killed': 'failed'})  

        smach.StateMachine.add("LOCKING", LockedPoint(myCom),
                            transitions={'locking': "LOCKING",
                                         'locking_complete': "SHOOTING",
                                         'aborted': 'aborted',
                                         'killed': 'failed'})  
        
        smach.StateMachine.add("SHOOTING", ShootTorpedo(myCom),
                                transitions={'shoot_again': "SEARCHCIRCLES",
                                             'shoot_complete': 'succeeded',
                                             'aborted': 'aborted',
                                             'killed': 'failed'})     
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/TORPEDO')
    introServer.start()
    
    sm.execute()
        
