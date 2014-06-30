import smach
import rospy
import smach_ros
from bbauv_msgs.msg import ControllerAction

class State(smach.State):
    def __init__(self, shared):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'failed'])
        self.shared = shared
        self.sm = smach.Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                                 connector_outcome='succeeded')
    
    def execute(self, userdata):
        with self.sm:
            smach.Sequence.add('DIVE', smach_ros.SimpleActionState(
                    'LocomotionServer',
                    ControllerAction,
                    goal=self.shared.build_goal(depth=0.6),
                    exec_timeout=rospy.Duration(15.0)
                ))
            smach.Sequence.add('FORWARD', smach_ros.SimpleActionState(
                    'LocomotionServer',
                    ControllerAction,
                    goal=self.shared.build_goal(fwd=10.0),
                    exec_timeout=rospy.Duration(30.0)
                ))

        res = self.sm.execute()
        rospy.loginfo('time taken for gate state: %s' % str((rospy.Time.now() -
                                                            self.shared.start_time).to_sec()))
        if res == 'aborted':
            return 'failed'
        return res
