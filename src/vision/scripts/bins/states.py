import rospy
import smach

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
            rospy.sleep(rospy.Duration(0.3))

        self.comms.register()
        if self.comms.isAlone:
            self.comms.inputHeading = self.comms.curHeading
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

                self.comms.sendMovement(f=1.0, sm=1.0,
                                        h=self.comms.inputHeading,
                                        blocking=False)

        # Reset waitingTimeout for next time
        self.waitingTimeout = self.defaultWaitingTime

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

    numTrials = 2
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
        self.comms.nearest = nearest

        dx = (closestCentroid[0] - self.centerX) / self.width
        dy = (closestCentroid[1] - self.centerY) / self.height

        if abs(dx) > self.maxdx or abs(dy) > self.maxdy:
            self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                    blocking=False)
            return 'centering'

        if self.trialsPassed == self.numTrials:
            self.comms.sendMovement(f=0.0, sm=0.0, h=self.comms.inputHeading,
                                    blocking=True)
            return 'centered'
        else:
            self.comms.sendMovement(f=0.0, sm=0.0, h=self.comms.inputHeading,
                                    blocking=True)
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

        adjustAngle = Utils.normAngle(self.comms.nearest['angle'] +
                                      self.comms.curHeading)
        self.comms.adjustAngle = adjustAngle
        self.comms.sendMovement(h=adjustAngle, blocking=True)
        return 'aligned'


class Fire(smach.State):
    sinkingDepth = 3.0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.sendMovement(h=self.comms.adjustAngle,
                                d=sinkingDepth, blocking=True)

        self.comms.drop()
        return 'completed'


def main():
    rospy.init_node('bins_node')
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
                                            'timeout':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER',
                               Center(myCom),
                               transitions={'centered':'ALIGN',
                                            'centering':'CENTER',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('ALIGN',
                               Align(myCom),
                               transitions={'aligned':'FIRE',
                                            'aligning':'ALIGN',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('FIRE',
                               Fire(myCom),
                               transitions={'completed':'succeeded',
                                            'aborted':'DISENGAGE'})

    sm.execute()
    rospy.signal_shutdown("lane_marker task ended")

