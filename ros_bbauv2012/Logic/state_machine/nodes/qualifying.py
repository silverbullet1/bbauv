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


class SmachSwitch(smach.State):
    def __init__(self, state=False):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        self.state = state
        
    def execute(self, userdata):
        rospy.set_param('/aggregator/inStateMachine', self.state)                 
        return 'succeeded'
        
class TopsideSwitch(smach.State):
    def __init__(self, state=True):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        self.state = state
                
    def execute(self, userdata):
        rospy.set_param('/aggregator/inTopside', self.state)                 
        return 'succeeded'

#        self.pub = rospy.Publisher('/controller_mode',controller_onoff,latch=True)
#        mode = controller_onoff()
#        mode.topside = False        
#        self.pub.publish(mode)

class NavigationSwitch(smach.State):
    def __init__(self, state=False):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        self.state = state
        
    def execute(self, userdata):
        rospy.set_param('/aggregator/inNavigation', self.state)                 
        return 'succeeded'

class TrackerSwitch(smach.State):
    def __init__(self, state=False):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        self.state = state
        
    def execute(self, userdata):
        rospy.set_param('/aggregator/inVisionTracking', self.state)                 
        return 'succeeded'

def shutdown():
    rospy.set_param('/aggregator/inStateMachine', False)                 

def main():
    rospy.init_node('bbauv_suavc_smach')

    sauvc = smach.StateMachine(outcomes=['succeeded','preempted'])

    with sauvc:
                                                                 
        smach.StateMachine.add('COUNTDOWN_START',Countdown(1.0),transitions={'succeeded':'SMACH_TRUE'})
        
        smach.StateMachine.add('SMACH_TRUE',SmachSwitch(True),transitions={'succeeded':'VISION_TRACK_TRUE'}) 
        smach.StateMachine.add('VISION_TRACK_TRUE',TrackerSwitch(False),transitions={'succeeded':'NAV_TRUE'})                                        
        smach.StateMachine.add('NAV_TRUE',NavigationSwitch(True),transitions={'succeeded':'TOPSIDE_FALSE'})                                       
        smach.StateMachine.add('TOPSIDE_FALSE',TopsideSwitch(False),transitions={'succeeded':'COUNTDOWN_MISSION'})  
                              
        smach.StateMachine.add('COUNTDOWN_MISSION',Countdown(240),transitions={'succeeded':'TOPSIDE_TRUE'})
        
        smach.StateMachine.add('TOPSIDE_TRUE',TopsideSwitch(True),transitions={'succeeded':'COUNTDOWN_END'})
        smach.StateMachine.add('COUNTDOWN_END',Countdown(0.1),transitions={'succeeded':'VISION_TRACK_FALSE'})
        smach.StateMachine.add('VISION_TRACK_FALSE',TrackerSwitch(False),transitions={'succeeded':'NAV_FALSE'})                                        
        smach.StateMachine.add('NAV_FALSE',NavigationSwitch(False),transitions={'succeeded':'SMACH_FALSE'})                  
        smach.StateMachine.add('SMACH_FALSE',SmachSwitch(False),transitions={'succeeded':'succeeded'})   
                                        
    sis = IntrospectionServer('my_introspserver', sauvc,'/SM_ROOT')
    sis.start()
    outcome = sauvc.execute()
    rospy.on_shutdown(shutdown)
        
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
