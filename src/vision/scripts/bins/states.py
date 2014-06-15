import rospy
import smach, smach_ros

from utils.utils import Utils
from comms import Comms
from vision import BinsVision

import time

""" The entry script and smach StateMachine for the task"""

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['started', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        self.comms.unregister()

        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.5))

        self.comms.register()
        rospy.sleep(rospy.Duration(1))
        if self.comms.isAlone:
            self.comms.inputHeading = self.comms.curHeading
        self.comms.sendMovement(d=self.comms.defaultDepth, blocking=True)
        return 'started'

class Search(smach.State):
    timeout = 200
    defaultWaitingTime = 2

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['foundBins',
                                             'timeout',
                                             'aborted'])
        self.comms = comms
        self.waitingTimeout = self.defaultWaitingTime

    def execute(self, userdata):
        start = time.time()

        while not self.comms.retVal or \
              len(self.comms.retVal['matches']) == 0:
            if self.comms.isKilled or self.comms.isAborted:
                self.comms.abortMission()
                return 'aborted'

            if time.time() - start > self.waitingTimeout:
                self.waitingTimeout = -1
                break

            rospy.sleep(rospy.Duration(0.3))

        start = time.time()
        if self.waitingTimeout < 0:
            # Waiting timeout, start searching pattern until timeout
            rospy.loginfo("Idling timeout, start searching")
            while (not self.comms.retVal or
                   len(self.comms.retVal['matches']) == 0):
                if self.comms.isKilled or self.comms.isAborted:
                    self.comms.abortMission()
                    return 'aborted'

                if (time.time() - start) > self.timeout:
                    self.comms.abortMission()
                    return 'aborted'
                
                rospy.sleep(rospy.Duration(0.3))
                #self.comms.sendMovement(f=0.0, sm=0.0,
                #                        h=self.comms.inputHeading,
                #                        blocking=False)

        # Reset waitingTimeout for next time
        self.waitingTimeout = self.defaultWaitingTime
        self.comms.searchComplete()
        return 'foundBins' 

class Center(smach.State):
    maxdx = 0.03
    maxdy = 0.03

    width = BinsVision.screen['width']
    height = BinsVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 1
    trialsPassed = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['matches']) == 0:
            return 'lost'

        matches = self.comms.retVal['matches']
        nearest = min(matches,
                      key=lambda m:
                      Utils.distBetweenPoints(m['centroid'],
                                              (self.centerX, self.centerY)))
        closestCentroid = nearest['centroid']

        dx = (closestCentroid[0] - self.centerX) / self.width
        dy = (closestCentroid[1] - self.centerY) / self.height

        if abs(dx) > self.maxdx or abs(dy) > self.maxdy:
            self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                    blocking=False)
            return 'centering'

        self.comms.sendMovement(f=0.0, sm=0.0, h=self.comms.inputHeading,
                                blocking=True)
        if self.trialsPassed == self.numTrials:
            self.comms.nearest = nearest
            self.trialsPassed = 0
            return 'centered'
        else:
            self.trialsPassed += 1
            return 'centering'

class Align(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['aligned',
                                             'aligning',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['matches']) == 0:
            return 'lost'

        #self.comms.sendMovement(d=self.comms.sinkingDepth,
        #                        f=-0.3,
        #                        blocking=True)

        # Align with the bins
        dAngle = Utils.toHeadingSpace(self.comms.nearest['angle'])
        adjustAngle = Utils.normAngle(dAngle + self.comms.curHeading)
        self.comms.adjustHeading = adjustAngle
        self.comms.sendMovement(h=adjustAngle, blocking=True)

        return 'aligned'

class CenterAgain(smach.State):
    maxdx = 0.03
    maxdy = 0.03

    width = BinsVision.screen['width']
    height = BinsVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 1
    trialsPassed = 0

    timeout = 5

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.start = time.time()
        while not self.comms.retVal or \
              len(self.comms.retVal['matches']) == 0:
            if time.time() - self.start > self.timeout:
                return 'lost'
            rospy.sleep(rospy.Duration(0.05))
            return 'centering'

        matches = self.comms.retVal['matches']
        nearest = min(matches,
                      key=lambda m:
                      Utils.distBetweenPoints(m['centroid'],
                                              (self.centerX, self.centerY)))
        closestCentroid = nearest['centroid']
        self.comms.nearest = nearest

        dx = (closestCentroid[0] - self.centerX) / self.width
        dy = (closestCentroid[1] - self.centerY) / self.height

        if abs(dx) > self.maxdx or abs(dy) > self.maxdy:
            self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                    d=self.comms.sinkingDepth,
                                    h=self.comms.adjustHeading,
                                    blocking=False)
            return 'centering'

        self.comms.sendMovement(f=0.0, sm=0.0,
                                d=self.comms.sinkingDepth,
                                h=self.comms.adjustHeading,
                                blocking=True)
        if self.trialsPassed == self.numTrials:
            self.trialsPassed = 0
            return 'centered'
        else:
            self.trialsPassed += 1
            return 'centering'

class Fire(smach.State):
    fireTimes = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'next',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.sendMovement(h=self.comms.adjustHeading,
                                d=self.comms.sinkingDepth, blocking=True)
        self.comms.drop()
        if self.fireTimes == 0:
            self.fireTimes += 1
            return 'next'
        else:
            return 'completed'

class Search2(smach.State):
    turnTime1 = 5
    turnTime2 = 7

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['foundBins',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        # Turn to the left and look for another bin
        self.comms.sendMovement(d=self.comms.defaultDepth, blocking=True)
        self.comms.sendMovement(h=Utils.normAngle(self.comms.adjustHeading-90),
                                blocking=True)
        self.comms.sendMovement(f=1.1, d=self.comms.defaultDepth, blocking=True)
        start = time.time()
        timeExceeded = False
        while (not self.comms.retVal or
               len(self.comms.retVal['matches']) == 0):
            if time.time() - start > self.turnTime1:
                timeExceeded = True
                break
            self.comms.sendMovement(f=0.3,
                                    d=self.comms.defaultDepth,
                                    blocking=False)

        if not timeExceeded:
            self.comms.adjustHeading = self.comms.curHeading
            return 'foundBins'

        # Turn to the right and look for another bin
        self.comms.sendMovement(h=Utils.normAngle(self.comms.adjustHeading+90),
                                d=self.comms.defaultDepth,
                                blocking=True)
        self.comms.sendMovement(f=3.5, d=self.comms.defaultDepth, blocking=True)
        start = time.time()
        while (not self.comms.retVal or
               len(self.comms.retVal['matches']) == 0):
            if time.time() - start > self.turnTime2:
                self.comms.abortMission()
                return 'lost'
            self.comms.sendMovement(f=-0.3,
                                    d=self.comms.defaultDepth,
                                    blocking=False)

        self.comms.adjustHeading = self.comms.curHeading
        return 'foundBins'

class Center2(smach.State):
    maxdx = 0.03
    maxdy = 0.03

    width = BinsVision.screen['width']
    height = BinsVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 2
    trialsPassed = 0

    timeout = 5

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.start = time.time()
        while not self.comms.retVal or \
              len(self.comms.retVal['matches']) == 0:
            if time.time() - self.start > self.timeout:
                return 'lost'
            rospy.sleep(rospy.Duration(0.05))
            return 'centering'

        matches = self.comms.retVal['matches']
        nearest = min(matches,
                      key=lambda m:
                      Utils.distBetweenPoints(m['centroid'],
                                              (self.centerX, self.centerY)))
        closestCentroid = nearest['centroid']

        dx = (closestCentroid[0] - self.centerX) / self.width
        dy = (closestCentroid[1] - self.centerY) / self.height

        if abs(dx) > self.maxdx or abs(dy) > self.maxdy:
            self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                    h=self.comms.adjustHeading,
                                    blocking=False)
            return 'centering'

        self.comms.sendMovement(f=0.0, sm=0.0,
                                h=self.comms.adjustHeading,
                                blocking=True)
        if self.trialsPassed == self.numTrials:
            self.comms.nearest = nearest
            self.trialsPassed = 0
            return 'centered'
        else:
            self.trialsPassed += 1
            return 'centering'


def main():
    rospy.init_node('bins')
    myCom = Comms()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENGAGE',
                               Disengage(myCom),
                               transitions={'started':'SEARCH',
                                            'killed':'killed'})
        smach.StateMachine.add('SEARCH',
                               Search(myCom),
                               transitions={'foundBins':'CENTER',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER',
                               Center(myCom),
                               transitions={'centered':'ALIGN',
                                            'centering':'CENTER',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('ALIGN',
                               Align(myCom),
                               transitions={'aligned':'CENTERAGAIN',
                                            'aligning':'ALIGN',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTERAGAIN',
                               CenterAgain(myCom),
                               transitions={'centered':'FIRE',
                                            'centering':'CENTERAGAIN',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('FIRE',
                               Fire(myCom),
                               transitions={'completed':'succeeded',
                                            'next':'SEARCH2',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('SEARCH2',
                               Search2(myCom),
                               transitions={'foundBins':'CENTER2',
                                            'lost':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER2',
                               Center2(myCom),
                               transitions={'centered':'ALIGN',
                                            'centering':'CENTER2',
                                            'lost':'DISENGAGE',
                                            'aborted':'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/PICKUP')
    introServer.start()

    sm.execute()
    rospy.signal_shutdown("lane_marker task ended")

