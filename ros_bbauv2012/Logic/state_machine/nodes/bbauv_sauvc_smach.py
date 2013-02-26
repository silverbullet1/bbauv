#!/usr/bin/env python

import roslib; roslib.load_manifest('state_machine')

import rospy
import smach
import smach_ros
from smach_ros import IntrospectionServer
from bbauv_msgs.msg import controller_onoff

class Countdown(smach.State):
    def __init__(self, sleep=1.0):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        self.sleep_time = sleep
    
    def execute(self, userdata):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
            if self.preempt_requested():
                self.services_preempt()
                return 'preempted'
        return 'succeeded'

class SmachSwitchTRUE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        
    def execute(self, userdata):
        rospy.set_param('/aggregator/inStateMachine', True)                 
        return 'succeeded'

class SmachSwitchFALSE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        
    def execute(self, userdata):
        rospy.set_param('/aggregator/inStateMachine', False)                 
        return 'succeeded'

class TopsideSwitchTRUE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        
    def execute(self, userdata):
        rospy.set_param('/aggregator/inTopside', True)                 
        return 'succeeded'

class TopsideSwitchFALSE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
#        self.pub = rospy.Publisher('/controller_mode',controller_onoff,latch=True)
#        mode = controller_onoff()
#        mode.topside = False        
#        self.pub.publish(mode)
                
    def execute(self, userdata):
    
        rospy.set_param('/aggregator/inTopside', False)         
        return 'succeeded'


def main():
    rospy.init_node('bbauv_suavc_smach')

    sauvc = smach.StateMachine(outcomes=['succeeded','preempted'])

    with sauvc:
        smach.StateMachine.add('COUNTDOWN_START',Countdown(1.0),transitions={'succeeded':'SMACH_TRUE'})
        smach.StateMachine.add('SMACH_TRUE',SmachSwitchTRUE(),transitions={'succeeded':'TOPSIDE_FALSE'})                                
        smach.StateMachine.add('TOPSIDE_FALSE',TopsideSwitchFALSE(),transitions={'succeeded':'COUNTDOWN_MISSION'})                        
        smach.StateMachine.add('COUNTDOWN_MISSION',Countdown(5.0),transitions={'succeeded':'TOPSIDE_TRUE'})
        smach.StateMachine.add('TOPSIDE_TRUE',TopsideSwitchTRUE(),transitions={'succeeded':'COUNTDOWN_END'})
        smach.StateMachine.add('COUNTDOWN_END',Countdown(0.1),transitions={'succeeded':'SMACH_FALSE'})        
        smach.StateMachine.add('SMACH_FALSE',SmachSwitchFALSE(),transitions={'succeeded':'succeeded'})                                
                                        
    sis = IntrospectionServer('my_introspserver', sauvc,'/SM_ROOT')
    sis.start()
    outcome = sauvc.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
