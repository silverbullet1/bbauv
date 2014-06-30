import smach
from utils.common import TimeoutState
import smach_ros
from bbauv_msgs.srv import set_controllerRequest, set_controller
import rospy

class State(smach.State):
    def __init__(self, shared):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failed'])
        self._shared = shared
        self.sm0 = smach.StateMachine(outcomes=['succeeded', 'preempted',
                                                'failed'])
        with self.sm0:
            smach.StateMachine.add('WAIT', TimeoutState(5),
                                   transitions={'succeeded' : 'INITPID',
                                                'preempted' : 'preempted'})
            smach.StateMachine.add('INITPID', smach_ros.ServiceState(
                    'set_controller_srv',
                    set_controller,
                    request=set_controllerRequest(forward=True, sidemove=True,
                                                  roll=True, heading=True,
                                                  pitch=True, navigation=False,
                                                  topside=False)
                ), transitions={'succeeded' : 'succeeded', 'aborted' : 'failed'})

    def request_preempt(self):
        self.sm0.request_preempt()

    def execute(self, userdata):
        res = self.sm0.execute()
        rospy.loginfo('time taken to init: %s' % str((rospy.Time.now() -
                                                  self._shared.start_time).to_sec()))
        return res
