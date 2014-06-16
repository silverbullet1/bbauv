import time

import rospy
import smach, smach_ros

from comms import Comms
from vision import PickupVision
from utils.utils import Utils

""" The entry script and smach StateMachine for the task"""

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['started', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))

        self.comms.register()
        if self.comms.isAlone:
            rospy.sleep(rospy.Duration(1))
            self.comms.inputHeading = self.comms.curHeading
        self.comms.sendMovement(d=self.comms.inputHeading)
        return 'started'

class SearchSite(smach.State):
    timeout = 10

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['aborted', 'lost', 'foundSite'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()

        while not self.comms.retVal or \
              len (self.comms.retVal['site']) < 1:
            if time.time() - start > self.timeout:
                self.comms.abortMission()
                return 'timeout'
            rospy.sleep(rospy.Duration(0.3))

        return 'foundSite'

class CenterSite(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 1
    trialPassed = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['site']) < 1:
            return 'lost'

        site = self.comms.retVal['site']

        dx = (site['centroid'][0] - self.centerX) / self.width
        dy = (site['centroid'][1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.sendMovement(f=0.0, sm=0.0, blocking=True)
            if self.trialPassed == self.numTrials:
                self.trialPassed = 0
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'


        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                blocking=False)
        return 'centering'
        

class Search(smach.State):
    timeout = 10

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['foundSamples',
                                             'timeout',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()

        while not self.comms.retVal or \
              len (self.comms.retVal['samples']) < 1:
            if time.time() - start > self.timeout:
                self.comms.abortMission()
                return 'timeout'
            rospy.sleep(rospy.Duration(0.3))

        return 'foundSamples'

class Center(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 1
    trialPassed = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['centroids']) < 1:
            return 'lost'

        samples = self.comms.retVal['centroids']
        closest = min(samples,
                      key=lambda c:
                      Utils.distBetweenPoints(c['centroid'],
                                              (self.centerX, self.centerY)))

        dx = (closest[0] - self.centerX) / self.width
        dy = (closest[1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.sendMovement(f=0.0, sm=0.0, blocking=True)
            if self.trialPassed == self.numTrials:
                self.trialPassed = 0
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'


        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                blocking=False)
        return 'centering'

class Approach(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'approaching',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.sendMovement(d=self.comms.sinkingDepth, blocking=True)

        return 'completed'

class Center2(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 3.0
    ycoeff = 2.5

    numTrials = 1
    trialPassed = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['centroids']) < 1:
            return 'lost'

        samples = self.comms.retVal['samples']
        closest = min(samples,
                      key=lambda c:
                      Utils.distBetweenPoints(c['centroid'],
                                              (self.centerX, self.centerY)))

        dx = (closest[0] - self.centerX) / self.width
        dy = (closest[1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.sendMovement(f=0.0, sm=0.0,
                                    d=self.comms.sinkingDepth,
                                    blocking=True)
            if self.trialPassed == self.numTrials:
                self.trialPassed = 0
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'

        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                d=self.comms.sinkingDepth,
                                blocking=False)
        return 'centering'

class Grab(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['grabbed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.grab()

        return 'grabbed'

class Surface(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.sendMovement(d=self.comms.defaultDepth)

        return 'completed'

class Navigate(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        #TODO: Navigate to the collection site

        return 'completed'

class Drop(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.drop()
        self.comms.taskComplete()

        return 'completed'


def main():
    rospy.init_node('pickup')
    myCom = Comms()    

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENGAGE',
                               Disengage(myCom),
                               transitions={'started':'SEARCH',
                                            'killed':'killed'})
        smach.StateMachine.add('SEARCHSITE',
                               SearchSite(myCom),
                               transitions={'foundSite':'CENTERSITE',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTERSITE',
                               CenterSite(myCom),
                               transitions={'centered':'SEARCH',
                                            'centering':'CENTERSITE',
                                            'lost':'SEARCHSITE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('SEARCH',
                               Search(myCom),
                               transitions={'foundSamples':'CENTER',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER',
                               Center(myCom),
                               transitions={'centered':'APPROACH',
                                            'centering':'CENTER',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('APPROACH',
                               Approach(myCom),
                               transitions={'completed':'GRAB',
                                            'approaching':'APPROACH',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER2',
                               Center2(myCom),
                               transitions={'centered':'GRAB',
                                            'centering':'CENTER2',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('GRAB',
                               Grab(myCom),
                               transitions={'grabbed':'SURFACE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('SURFACE',
                               Surface(myCom),
                               transitions={'completed':'NAVIGATE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('NAVIGATE',
                               Navigate(myCom),
                               transitions={'completed':'DROP',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('DROP',
                               Drop(myCom),
                               transitions={'completed':'succeeded',
                                            'aborted':'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/PICKUP')
    introServer.start()

    sm.execute()   

