import rospy
import smach, smach_ros

from comms import Comms
from vision import LaneMarkerVision

import time

""" The entry script and smach StateMachine for the task"""

class Disengage(smach.State):
    def __init__(self, comms):
        smach.StateMachine.__init__(outcomes=['started', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))

        self.comms.register()
        return 'started'

class Search(smach.State):
    timeout = 7

    def __init__(self, comms):
        smach.StateMachine.__init__(outcomes=['foundLanes',
                                              'timeout',
                                              'aborted'])
        self.comms = comms

    def execute(self, userdata):
        start = time.time()

        while len(self.comms.retVal['foundLines']) == 0:
            if self.comms.isKilled or \
               self.comms.isAborted or \
               (time.time() - start) > self.timeout:
                return 'aborted'
            rospy.sleep(rospy.Duration(0.3))

        return 'foundLanes'

class Stablize(smach.State):
    maxdx = 0.2
    maxdy = 0.2
    width = LaneMarkerVision.screen['width']
    height = LaneMarkerVision.screen['height']

    def __init__(self, comms):
        smach.StateMachine.__init__(outcomes=['stablized',
                                              'lost',
                                              'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'

        centroid = self.comms.retVal['centroid']
        dX = (centroid[0] - self.width/2) / self.width
        dY = (centroid[1] - self.height/2) / self.height
        rospy.loginfo("x-off: %lf, y-off: %lf", dX, dY)

        if abs(dX) < self.maxdx and abs(dY) < self.maxdy:
            return 'stablized'

        
def main():
    rospy.init_node('lane_marker_node')
    myCom = Comms()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENAGE',
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
                               transitions={'stablized':'',
                                            'lost':'',
                                            'aborted':'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/LANE_MARKER')
    introServer.start()

    sm.execute()
