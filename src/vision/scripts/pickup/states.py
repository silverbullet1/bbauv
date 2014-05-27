import time

import rospy
import smach

from comms import Comms
from vision import PickupVision
from utils.utils import Utils

""" The entry script and smach StateMachine for the task"""

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(outcomes=['started', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))

        self.comms.register()
        return 'started'

class Search(smach.State):
    timeout = 10

    def __init__(self, comms):
        smach.State.__init__(outcomes=['foundSamples',
                                       'timeout',
                                       'aborted'])
        self.comms = comms

    def execute(self, userdata):
        start = time.time()

        while not self.comms.retVal or \
              len (self.comms.retVal['centroids']) < 1:
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
    xcoeff = 2.0
    ycoeff = 1.5

    def __init__(self, comms):
        smach.State.__init__(outcomes=['centered',
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
           len(self.comms.rectVal['centroid']) < 1:
            return 'lost'

        centroids = self.comms.retVal['centroids']
        closest = min(centroids,
                      key=lambda c:
                      Utils.distBetweenPoints(c, (self.centerX, self.centerY)))

        dx = (closest - self.centerX) / self.width
        dy = (closest - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                    blocking=False)
            return 'centering'

        self.comms.sendMovement(f=0.0, sm=0.0)
        return 'centered'

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

        return 'completed'

class Grab(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['grabbed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

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

        return 'complete'


def main():
    rospy.init_node('pickup_node')
    myCom = Comms()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENGAGE',
                               Disengage(myCom),
                               transitions={'started':'SEARCH',
                                            'killed':'killed'})
        smach.StateMachine.add('SEARCH',
                               Search(myCom),
                               transitions={'foundSamples':'CENTER',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER',
                               Center(myCom),
                               transitions={'centered':'',
                                            'centering':'CENTER',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})

    #introServer = smach_ros.IntrospectionServer('mission_server',
    #                                            sm,
    #                                            '/MISSION/PICKUP')
    #introServer.start()

    sm.execute()

