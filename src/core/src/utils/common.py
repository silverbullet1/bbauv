import smach
import rospy
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import ControllerGoal

class Shared(object):
    def __init__(self):
        self.yaw = None
        self.depth = None
        self.start_time = None
        self.absolute_pose = Odometry()

    def build_goal(self, fwd=0.0, sidem=0.0, heading=None, depth=None):
        forward_setpoint = fwd
        sidemove_setpoint = sidem
        heading_setpoint = heading or self.yaw
        depth_setpoint = depth or self.depth

        return ControllerGoal(forward_setpoint=forward_setpoint,
                              sidemove_setpoint=sidemove_setpoint,
                              heading_setpoint=heading_setpoint,
                              depth_setpoint=depth_setpoint)

class TimeoutState(smach.State):
    def __init__(self, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.timeout = timeout

    def execute(self, userdata):
        start = rospy.Time.now()
        while True:
            if self.preempt_requested():
                return 'preempted'
            if rospy.Time.now() - start >= rospy.Duration(self.timeout):
                return 'succeeded'
            rospy.sleep(0.1)
