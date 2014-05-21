import time

import rospy
import smach, smach_ros

from comms import Comms

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
    timeout = 10

    def __init__(self, comms):
        smach.StateMachine.__init__(outcomes=['foundSamples',
                                              'timeout',
                                              'aborted'])
        self.comms = comms

    def execute(self, userdata):
        start = time.time()

        while not self.comms.retVal or \
              len (self.comms.retVal['centroids']) < 1:
            if time.time() - start > self.timeout:
                self.comms.isAborted = True
                return 'timeout'
            rospy.sleep(rospy.Duration(0.3))


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
                               transitions={'foundSamples':'',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/PICKUP')
    introServer.start()

    sm.execute()

