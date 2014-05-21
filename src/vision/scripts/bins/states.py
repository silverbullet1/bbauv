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
            rospy.sleep(rospy.Duration(0.3))

        self.comms.register()
        return 'started'

class Search(smach.State):
    timeout = 1000

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['foundBins',
                                             'timeout',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        start = time.time()

        while not self.comms.retVal or \
              len(self.comms.retVal['matches']) == 0:
            if self.comms.isKilled or self.comms.isAborted:
                return 'aborted'
            if time.time() - start > self.timeout:
                self.comms.isAborted = True
                return 'aborted'
            rospy.sleep(rospy.Duration(0.3))

        return 'foundBins' 

class Center(smach.State):
    maxdx = 0.03
    maxdy = 0.03

    width = BinsVision.screen['width']
    height = BinsVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    xcoeff = 2.0
    ycoeff = 1.5

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['matches']) == 0:
            return 'lost'

        matches = self.comms.retVal['matches']
        closestCentroid = None
        closestDist = 1000
        for match in matches:
            dist = Utils.distBetweenPoints(match['centroid'],
                                           (self.centerX, self.centerY))
            if dist < closestDist:
                closestCentroid = match['centroid']
                closestDist = dist

        dx = (closestCentroid[0] - self.centerX) / self.width
        dy = (closestCentroid[1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                    blocking=False)
            return 'centering'

        self.comms.sendMovement(f=0.0, sm=0.0)
        return 'centered'

class Fire(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            return 'aborted'

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
                               transitions={'centered':'FIRE',
                                            'centering':'CENTER',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('FIRE',
                               Fire(myCom),
                               transitions={'completed':'succeeded',
                                            'aborted':'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/BINS')
    introServer.start()

    sm.execute()
    rospy.signal_shutdown("lane_marker task ended")

