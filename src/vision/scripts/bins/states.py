import rospy

import smach, smach_ros

from comms import Comms

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
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['foundBins',
                                              'timeout',
                                              'aborted'])
        self.comms = comms

    def execute(self, userdata):
        while not self.comms.retVal or \
              len(self.comms.retVal['centroids']) == 0:
            if self.comms.isKilled or self.comms.isAborted:
                return 'aborted'
            rospy.sleep(rospy.Duration(0.3))

        return 'foundBins' 

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
                               transitions={'foundBins':'SEARCH',
                                            'timeout':'SEARCH',
                                            'aborted':'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/BINS')
    introServer.start()

    sm.execute()

