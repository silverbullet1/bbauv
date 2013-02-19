#!/usr/bin/env python

import roslib; roslib.load_manifest('state_machine')

import rospy
import smach
from smach import Concurrence
from smach_ros import ServiceState,IntrospectionServer,SimpleActionState, MonitorState

class Countdown(smach.state):
    def __init__(self, outcomes=['succeeded','preempted']):
        self.sleep_time = 3.0
    
    def execute(self,userdata):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
            if self.preempt_requested():
                self.services_preempt()
                return 'preempted'
        return 'succeeded'

class TopsideSwitch(smach.state):
    def __init__(self,outcomes=['succeeded','preempted']):
        self.inTopside = inTopside
    
    def execute(self):
        return 'succeeded'
        
class Line_Follower(smach.state):
    def __init__(self,outcomes=['succeeded','preempted']):
        
    
    def execute(self):
        return 'succeeded'
        


def main():
    rospy.init_node('smach_usecase_executive')

    sauvc = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sauvc:
        smach.StateMachine.add('COUNTDOWN',Countdown,transitions={'succeeded':'succeeded'})
        
        
    sis = IntrospectionServer('my_introspserver', sm_root,'/SM_ROOT')
    sis.start()
    outcome = sm_root.execute()

    rospy.spin()

if __name__ == '__main__':
    main()
