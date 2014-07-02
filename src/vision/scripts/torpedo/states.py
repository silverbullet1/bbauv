#!/usr/bin/env python

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
from front_commons.frontCommsVision import FrontCommsVision as vision

# Globals 
locomotionGoal = None 

class Disengage(smach.State):
    global missionStart

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        self.comms.state = "DISENGAGE"

        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if self.comms.isAlone:
            self.comms.register()
            self.comms.regCompass()
            self.comms.registerSonar()

            rospy.sleep(rospy.Duration(0.5))
            self.comms.inputHeading = self.comms.curHeading
            rospy.loginfo("Starting Torpedo")


        self.comms.missionStart = time.time()
        self.comms.curTime = time.time()
        self.comms.sendMovement(depth=self.comms.defaultDepth,
                                heading=self.comms.curHeading,
                                blocking=True)
        
        return 'start_complete'

class FollowSonar(smach.State):
    completeBoardArea = 11000
    forward_setpoint = 0.8
    sonarDist = 3.0

    timeout = 300

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['sonar_complete', 'following_sonar', 'aborted', 'killed'])
        self.comms = comms        
        rospy.sleep(duration=0.8)
        
    def execute(self, ud):
        self.comms.state = "SONARSONAR"

        if self.comms.sonarBearing > self.sonarDist or \
            self.comms.boardArea < self.completeBoardArea:
            rospy.loginfo("Sonar bearing {}".format(self.comms.sonarBearing))
            self.comms.curHeading = self.comms.heading + (self.comms.sonarBearing*-1.0)
            if self.comms.sonarBearing < -10 or \
                self.comms.sonarBearing >  10:
                sidemove = 1.0 if self.comms.sonarBearing < 0 else -1.0

                self.comms.sendMovement(forward=0.0,
                                        heading=self.comms.curHeading,
                                        sidemove=sidemove,
                                        blocking=True)
            self.comms.sendMovement(forward=0.8, blocking=True)
            return 'following_sonar'
        else:
            self.comms.curHeading = self.comms.heading
            return 'sonar_complete'

class SearchCircles(smach.State):
    timeout = 300
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchCircles_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
            
    def execute(self, ud):
        self.comms.state = "SEARCHING"
        self.comms.isMovingState = False

        start = time.time()
        curTime = time.time()
        
        while not self.comms.foundCircles and not self.comms.foundSomething: 
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 

            # if not self.comms.foundCircles and self.comms.foundSomething:
            #     if self.comms.foundCount < 50:
            #         # Search pattern 
            # self.comms.sendMovement(forward=0.2,
            #                         sidemove=-0.4, timeout=0.5, blocking=False)
            # self.comms.sendMovement(forward=0.2,
            #                         sidemove=0.4, timeout=0.5, blocking=False)
            #     else:
            #         return 'lost'

            self.comms.sendMovement(forward=0.2, timeout=0.6, blocking=False)

            rospy.sleep(rospy.Duration(0.3))   

        rospy.loginfo("Search circles time {}".format(time.time()-self.comms.curTime))
        self.comms.curTime = time.time()
        return 'searchCircles_complete'

# Align with the center of the green board 
class AlignBoard(smach.State):
    timeout = 300

    forward_setpoint = 0.50
    halfCompleteArea = 45000
    completeArea = 65000

    deltaXMult = 4.9
    deltaYMult = 1.6
    headingMult = 1.5

    skew = 1.00

    # completeArea = self.comms.movementParams['boardArea']
    # deltaXMult = self.comms.movementParams['alignDeltaX']
    # deltaYMult = self.comms.movementParams['alignDeltaY']

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['alignBoard_complete', 'aligning_board', 'aborted', 'killed'])
        self.comms = comms

    def execute(self, ud):
        self.comms.state = "BOARDBOARD"
        self.comms.isMovingState = False
        curTime = time.time()

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

        if self.comms.boardArea > self.completeArea and self.comms.skew < 0.35:
            self.comms.curHeading = self.comms.heading

            rospy.loginfo("Align Board time {}".format(time.time()-self.comms.curTime))
            self.comms.curTime = time.time()
            return 'alignBoard_complete'

        if abs(self.comms.boardDeltaY) > 0.035:
            # if self.comms.boardArea > self.halfCompleteArea:
            #     self.deltaYMult = 0.2
            #     self.deltaXMult = 3.0

            self.comms.defaultDepth = self.comms.depth + self.comms.boardDeltaY*self.deltaYMult
            # Prevent surfacing
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = self.comms.depthFromMission

        # if self.comms.boardArea > self.halfCompleteArea:
        #     self.forward_setpoint = 0.30

        if abs(self.comms.skew) < self.skew:
            self.comms.curHeading = self.comms.heading + abs(self.comms.skew) * self.headingMult
                
        # Side move and keep heading
        self.comms.sendMovement(forward=self.forward_setpoint, 
                                sidemove=self.comms.boardDeltaX * self.deltaXMult, 
                                heading=self.comms.curHeading, 
                                depth=self.comms.defaultDepth, timeout=0.4, 
                                blocking=False)

        return 'aligning_board'

# Just move forward 
class MoveForward(smach.State):

    forward_setpoint = 0.43
    completeRadius = 55
    lostCount = 0

    deltaXMult = 4.5
    deltaYMult = 1.5

    # completeRadius = self.comms.movementParams['forwardRadius']
    # deltaXMult = self.comms.movementParams['forwardDeltaX']
    # deltaYMult = self.comms.movementParams['forwardDeltaY']
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forwarding', 'forward_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):    
        self.comms.state = "MOVING"
        self.comms.isMovingState = True
        curTime = time.time()

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted' 
        
        if self.lostCount > 40:
            return 'lost'
        
        if not self.comms.foundCircles:
            self.lostCount = self.lostCount + 1
                    
        if self.comms.numShoot == 1:
            self.completeRadius = 45
            # self.deltaXMult = 0.8

        if self.comms.radius > self.completeRadius:
            rospy.loginfo("Move circles time {}".format(time.time()-self.comms.curTime))
            self.comms.curTime = time.time()

            return 'forward_complete'
        
        if abs(self.comms.deltaY) > 0.040:
            self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*self.deltaYMult
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
    deltaXMult = 3.8
    deltaYMult = 2.3

    centeringCount = 0
    halfCompleteRadius = 65
    completeRadius = 91
    forward_half = 0.23
    forward_setpoint = 0.22

    centering = 3

    # deltaXMult = self.comms.movementParams['centerDeltaX']
    # deltaYMult = self.comms.movementParams['centerDeltaY']
    # completeRadius = self.comms.movementParams['completeRadius']

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        self.comms.state = "CENTERING"
        self.comms.isMovingState = True
        curTime = time.time()

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'

        if self.comms.numShoot == 1:
            self.completeRadius = 86

        if self.comms.radius > self.completeRadius:
            sidemove_setpoint = self.comms.deltaX * 1.3
            self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*0.46

            self.comms.sendMovement(forward = 0.0, 
                            sidemove = sidemove_setpoint,
                            depth = self.comms.defaultDepth, timeout=0.4, 
                            blocking = False)

            if abs(self.comms.deltaX) < 0.06 and abs(self.comms.deltaY) < 0.06:
                self.centeringCount += 1

            if self.comms.numShoot == 1:
                self.centering = 5
            if self.centeringCount > self.centering:
                self.centeringCount = 0

                rospy.loginfo("Centering time {}".format(time.time()-self.comms.curTime))
                self.comms.curTime = time.time()
                return 'centering_complete'
        else: 
            if abs(self.comms.deltaY) > 0.040:
                self.comms.defaultDepth = self.comms.depth + self.comms.deltaY*self.deltaYMult
                if self.comms.defaultDepth < 0.1:
                    self.comms.defaultDepth = self.comms.depthFromMission
            
            sidemove_setpoint = 0.0
            if abs(self.comms.deltaX) > 0.040:
                sidemove_setpoint = self.comms.deltaX * self.deltaXMult

            # Sidemove and center
            self.comms.sendMovement(forward = self.forward_setpoint,
                                    sidemove = sidemove_setpoint,
                                    depth = self.comms.defaultDepth, timeout=0.4,
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
            
                self.comms.sendMovement(forward = 0.20,
                                        sidemove = self.comms.deltaX * self.deltaXMult,
                                        depth = self.comms.defaultDepth,
                                        blocking = False)


        return 'locking'

# Shoot torpedo forward
class ShootTorpedo(smach.State):

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_again', 'shoot_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        self.comms.state = "SHOOT TO KILL"
        self.comms.isMovingState = True

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        if self.comms.numShoot == 0:
            rospy.loginfo("First shoot")
            # self.comms.sendMovement(forward = 0.0,
            #                 sidemove = self.comms.torpedoOffset,
            #                 depth = self.comms.defaultDepth,
            #                 blocking = True)    

            self.comms.shootTopTorpedo()
            rospy.loginfo("Big circle {}".format(time.time()-self.comms.missionStart))
            # self.comms.shootBotTorpedo()
            # self.comms.taskComplete()
            # return 'shoot_again'

            # self.comms.sendMovement(forward = -1.5,
            #                         sidemove = 0.0,
            #                         depth = self.comms.defaultDepth,
            #                         blocking = True)
        elif self.comms.numShoot == 1:
            rospy.loginfo("Second shoot")
            # self.comms.sendMovement(forward = 0.0,
            #         sidemove = self.comms.torpedoOffset,
            #         depth = self.comms.defaultDepth,
            #         blocking = True)  
            self.comms.shootBotTorpedo()
        
        # Reset parameters
        self.comms.centroidToShoot = None
        self.comms.numShoot = self.comms.numShoot + 1

        if self.comms.numShoot == 2:
            self.comms.taskComplete()
            rospy.loginfo("Completed task {}".format(time.time()-self.comms.missionStart))
            return 'shoot_complete'

        self.comms.state = "MOVING UP"
        self.comms.defaultDepth = 1.30
        self.comms.sendMovement(forward=-1.0, sidemove=0.0,
                                depth=self.comms.defaultDepth,
                                blocking=True)
       
        return 'shoot_again'

def main():
    rospy.init_node('torpedo_lynnette_awesomeness', anonymous=False)
    rosRate = rospy.Rate(30)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'killed', 'failed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "ALIGNBOARD",
                                         'killed': 'failed'})      
        
        smach.StateMachine.add("FOLLOWSONAR", FollowSonar(myCom),
                               transitions={'following_sonar': "FOLLOWSONAR",
                                           'sonar_complete': "ALIGNBOARD",
                                           'aborted': 'killed',
                                           'killed': 'failed'})

        smach.StateMachine.add("ALIGNBOARD", AlignBoard(myCom),
                                transitions={'aligning_board': "ALIGNBOARD",
                                            'alignBoard_complete': "MOVEFORWARD",
                                            'aborted': 'killed', 
                                            'killed': 'failed'
                                })
    
        smach.StateMachine.add("SEARCHCIRCLES", SearchCircles(myCom),
                                transitions={'searchCircles_complete': "ALIGNBOARD",
                                             'lost': 'failed',
                                             'aborted': 'killed',
                                             'killed': 'failed'})      
        
        smach.StateMachine.add("MOVEFORWARD", MoveForward(myCom),
                                transitions={'forwarding': "MOVEFORWARD",
                                             'forward_complete': "CENTERING",
                                             'lost': "SEARCHCIRCLES",
                                             'aborted': 'killed',
                                             'killed': 'failed'})       
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "SHOOTING",
                                             'lost': "SEARCHCIRCLES",
                                             'aborted': 'killed',
                                             'killed': 'failed'})  

        smach.StateMachine.add("LOCKING", LockedPoint(myCom),
                            transitions={'locking': "LOCKING",
                                         'locking_complete': "SHOOTING",
                                         'aborted': 'killed',
                                         'killed': 'failed'})  
        
        smach.StateMachine.add("SHOOTING", ShootTorpedo(myCom),
                                transitions={'shoot_again': "SEARCHCIRCLES",
                                             'shoot_complete': 'succeeded',
                                             'aborted': 'killed',
                                             'killed': 'failed'})     
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/TORPEDO')
    introServer.start()
    
    sm.execute()
        
