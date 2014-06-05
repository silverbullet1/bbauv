import rospy
import smach
import numpy as np

from comms import Comms
from utils.utils import Utils
from vision import LaneMarkerVision

import time
import math
from collections import deque

""" The entry script and smach StateMachine for the task"""

class MedianFilter:
    staleDuration = 5.0

    def __init__(self, sampleWindow=30):
        self.samples = deque()
        self.sampleWindow = sampleWindow
        self.lastSampled = time.time()

    def newSample(self, sample):
        curTime = time.time()
        # Discard previous samples if we only sampled them a long time ago
        if (curTime - self.lastSampled) > self.staleDuration:
            self.samples = deque()

        self.lastSampled = curTime
        if len(self.samples) >= self.sampleWindow:
            self.samples.popleft()
        self.samples.append(sample)

    def getMedian(self):
        return np.mean(self.samples)

    def getVariance(self):
        if len(self.samples) >= self.sampleWindow:
            return np.var(self.samples)
        else:
            return 999 # Just a big value

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['started', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        self.comms.unregister()

        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))

        self.comms.register()
        if self.comms.isAlone:
            self.comms.inputHeading = self.comms.curHeading
        self.comms.sendMovement(d=self.comms.defaultDepth,
                                h=self.comms.inputHeading,
                                blocking=True)
        return 'started'

class Search(smach.State):
    timeout = 35
    defaultWaitingTime = 1

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['foundLanes',
                                             'timeout',
                                             'aborted'])
        self.comms = comms
        self.waitingTimeout = self.defaultWaitingTime

    def execute(self, userdata):
        start = time.time()

        while (not self.comms.retVal or
               len(self.comms.retVal['foundLines']) == 0):
            # Waiting to see if lanes found until waitingTimeout
            if self.comms.isKilled or self.comms.isAborted:
                self.comms.abortMission()
                return 'aborted'

            if (time.time() - start) > self.waitingTimeout:
                self.waitingTimeout = -1
                break

            rospy.sleep(rospy.Duration(0.3))

        start = time.time()
        if self.waitingTimeout < 0:
            # Waiting timeout, start searching pattern until timeout
            rospy.loginfo("Idling timeout, start searching")

            while (not self.comms.retVal or
                   len(self.comms.retVal['foundLines']) == 0):
                if self.comms.isKilled or self.comms.isAborted:
                    self.comms.abortMission()
                    return 'aborted'

                if (time.time() - start) > self.timeout:
                    self.comms.abortMission()
                    return 'aborted'

                self.comms.sendMovement(f=2.0, sm=0.0,
                                        h=self.comms.inputHeading,
                                        blocking=False)

        # Reset waitingTimeout for next time
        self.waitingTimeout = self.defaultWaitingTime
        return 'foundLanes'

class Stablize(smach.State):
    maxdx = 0.05
    maxdy = 0.05
    width = LaneMarkerVision.screen['width']
    height = LaneMarkerVision.screen['height']

    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 2
    trialsPassed = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['stablized',
                                             'stablizing',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['foundLines']) == 0:
            return 'lost'

        centroid = self.comms.retVal['centroid']
        dX = (centroid[0] - self.width/2) / self.width
        dY = (centroid[1] - self.height/2) / self.height
        rospy.loginfo("x-off: %lf, y-off: %lf", dX, dY)

        if abs(dX) < self.maxdx and abs(dY) < self.maxdy:
            self.comms.sendMovement(f=0.0, sm=0.0,
                                    h=self.comms.inputHeading, blocking=True)
            if self.trialsPassed == self.numTrials:
                self.trialsPassed = 0
                return 'stablized'
            else:
                self.trialsPassed += 1
                return 'stablizing'

        f_setpoint = math.copysign(self.ycoeff * abs(dY), -dY)
        sm_setpoint = math.copysign(self.xcoeff * abs(dX), dX)
        self.comms.sendMovement(f=f_setpoint, sm=sm_setpoint,
                                h=self.comms.inputHeading, blocking=False)
        return 'stablizing'


class Align(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['aligned',
                                             'aligning',
                                             'lost',
                                             'aborted'])
        self.comms = comms
        self.angleSampler = MedianFilter(sampleWindow=50)

    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['foundLines']) == 0:
               return 'lost'

        lines = self.comms.retVal['foundLines']
        if len(lines) == 1 or self.comms.expectedLanes == 1:
            if Utils.angleDif(lines[0]['angle'], lines[0]['testAngle']) < 5:
                self.angleSampler.newSample(lines[0]['angle'])
                rospy.loginfo(Utils.normAngle(
                    Utils.toHeadingSpace(lines[0]['angle'])))
        elif len(lines) >= 2:
            if self.comms.chosenLane == self.comms.LEFT:
                if Utils.angleDif(lines[0]['angle'], lines[0]['testAngle']) < 5:
                    self.angleSampler.newSample(lines[0]['angle'])
                    rospy.loginfo(Utils.normAngle(
                        Utils.toHeadingSpace(lines[0]['angle'])))
            elif self.comms.chosenLane == self.comms.RIGHT:
                if Utils.angleDif(lines[1]['angle'], lines[1]['testAngle']) < 5:
                    self.angleSampler.newSample(lines[1]['angle'])
                    rospy.loginfo(Utils.normAngle(
                        Utils.toHeadingSpace(lines[1]['angle'])))
            else:
                rospy.loginfo("Something goes wrong with chosenLane")

        variance = self.angleSampler.getVariance()
        rospy.loginfo("Variance: {}".format(variance))
        if (variance < 5.0):
            dAngle = Utils.toHeadingSpace(self.angleSampler.getMedian())
            adjustHeading = Utils.normAngle(self.comms.curHeading + dAngle)

            self.comms.sendMovement(h=adjustHeading, blocking=True)
            self.comms.sendMovement(h=adjustHeading, blocking=True)
            self.comms.adjustHeading = adjustHeading
            return 'aligned'
        else:
            rospy.sleep(rospy.Duration(0.05))
            return 'aligning'

class Center(smach.State):
    maxdx = 0.03
    maxdy = 0.05
    width = LaneMarkerVision.screen['width']
    height = LaneMarkerVision.screen['height']

    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 2
    trialsPassed = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['foundLines']) == 0:
            return 'lost'

        centroid = self.comms.retVal['centroid']
        dX = (centroid[0] - self.width/2) / self.width
        dY = (centroid[1] - self.height/2) / self.height
        rospy.loginfo("x-off: %lf, y-off: %lf", dX, dY)

        if abs(dX) < self.maxdx and abs(dY) < self.maxdy:
            self.comms.sendMovement(f=0.0, sm=0.0,
                                    h=self.comms.adjustHeading, blocking=True)
            if self.trialsPassed == self.numTrials:
                self.trialsPassed = 0
                return 'centered'
            else:
                self.trialsPassed += 1
                return 'centering'

        f_setpoint = math.copysign(self.ycoeff * abs(dY), -dY)
        sm_setpoint = math.copysign(self.xcoeff * abs(dX), dX)
        self.comms.sendMovement(f=f_setpoint, sm=sm_setpoint,
                                h=self.comms.adjustHeading, blocking=False)
        return 'centering'


class Forward(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            self.comms.abortMission()
            return 'aborted'

        self.comms.sendMovement(f=3.0, h=self.comms.adjustHeading,
                                blocking=True)
        self.comms.taskComplete(heading=self.comms.adjustHeading)
        return 'completed'

def main():
    rospy.init_node('lane_marker')
    myCom = Comms()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENGAGE',
                               Disengage(myCom),
                               transitions={'started':'SEARCH',
                                            'killed':'killed'})
        smach.StateMachine.add('SEARCH',
                               Search(myCom),
                               transitions={'foundLanes':'STABLIZE',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('STABLIZE',
                               Stablize(myCom),
                               transitions={'stablized':'ALIGN',
                                            'stablizing':'STABLIZE',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('ALIGN',
                               Align(myCom),
                               transitions={'aligned':'CENTER',
                                            'aligning':'ALIGN',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER',
                               Center(myCom),
                               transitions={'centered':'FORWARD',
                                            'centering':'CENTER',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('FORWARD',
                               Forward(myCom),
                               transitions={'completed':'DISENGAGE',
                                            'aborted':'DISENGAGE'})

    sm.execute()
