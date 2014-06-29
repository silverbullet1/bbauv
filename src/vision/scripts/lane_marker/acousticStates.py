import rospy
import smach, smach_ros
import numpy as np

from comms import Comms
from states import Search, Stablize, Align, Center, Forward
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
        self.comms.detectingBox = True

        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.5))

        self.comms.register()
        if self.comms.isAlone:
            rospy.sleep(rospy.Duration(1))
            self.comms.inputHeading = self.comms.curHeading
            self.comms.sendMovement(d=self.comms.defaultDepth,
                                    h=self.comms.inputHeading,
                                    blocking=True)
        self.comms.retVal = None
        return 'started'

class CenterBox(smach.State):
    maxdx = 0.05
    maxdy = 0.05
    width = LaneMarkerVision.screen['width']
    height = LaneMarkerVision.screen['height']

    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 1
    trialsPassed = 0

    timeout = 50

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

        start = time.time()
        while not self.comms.retVal or \
              len(self.comms.retVal['foundLines']) == 0:
            if time.time() - start > self.timeout:
                return 'lost'
            rospy.sleep(rospy.Duration(0.1))

        self.comms.searchComplete()

        centroid = self.comms.retVal['box']['centroid']
        dX = (centroid[0] - self.width/2) / self.width
        dY = (centroid[1] - self.height/2) / self.height
        rospy.loginfo("x-off: %lf, y-off: %lf", dX, dY)

        if abs(dX) < self.maxdx and abs(dY) < self.maxdy:
            self.comms.sendMovement(f=0.0, sm=0.0,
                                    h=self.comms.adjustHeading, blocking=True)
            if self.trialsPassed == self.numTrials:
                self.trialsPassed = 0
                self.comms.detectingBox = False
                return 'centered'
            else:
                self.trialsPassed += 1
                return 'centering'

        f_setpoint = math.copysign(self.ycoeff * abs(dY), -dY)
        sm_setpoint = math.copysign(self.xcoeff * abs(dX), dX)
        self.comms.sendMovement(f=f_setpoint, sm=sm_setpoint,
                                h=self.comms.adjustHeading, blocking=False)
        return 'centering'

class AlignBoxLane(smach.State):
    width = LaneMarkerVision.screen['width']
    height = LaneMarkerVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    timeout = 3

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['aligned',
                                             'aligning',
                                             'lost',
                                             'aborted'])
        self.comms = comms
        self.angleSampler = MedianFilter(sampleWindow=30)

    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()
        while not self.comms.retVal or \
              len(self.comms.retVal['foundLines']) == 0:
            if time.time() - start > self.timeout:
                return 'lost'
            rospy.sleep(rospy.Duration(0.1))

        # Calculate angle between box and lane
        boxCentroid = (self.centerX, self.centerY)
        laneCentroid = self.comms.retVal['foundLines'][0]['pos']
        boxLaneAngle = math.atan2(laneCentroid[1] - boxCentroid[1],
                                  laneCentroid[0] - boxCentroid[0])
        self.angleSampler.newSample(math.degrees(boxLaneAngle))

        variance = self.angleSampler.getVariance()
        rospy.loginfo("Variance: {}".format(variance))
        if (variance < 5.0):
            dAngle = Utils.toHeadingSpace(self.angleSampler.getMedian())
            adjustHeading = Utils.normAngle(self.comms.curHeading + dAngle)

            self.comms.sendMovement(f=0.2, h=adjustHeading, blocking=True)
            self.comms.inputHeading = adjustHeading
            return 'aligned'
        else:
            rospy.sleep(rospy.Duration(0.05))
            return 'aligning'


def main():
    rospy.init_node('lane')
    myCom = Comms(True)

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENGAGE',
                               Disengage(myCom),
                               transitions={'started':'CENTER',
                                            'killed':'killed'})
        smach.StateMachine.add('CENTERBOX',
                               Center(myCom),
                               transitions={'centered':'ALIGNBOXLANE',
                                            'centering':'CENTERBOX',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('ALIGNBOXLANE',
                               AlignBoxLane(myCom),
                               transitions={'aligned' : 'SEARCH',
                                            'aligning': 'ALIGNBOXLANE',
                                            'lost' : 'SEARCH',
                                            'aborted': 'DISENGAGE'})
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

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/LANE')
    introServer.start()

    sm.execute()
    rospy.signal_shutdown('lane task quit')
