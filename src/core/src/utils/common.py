import smach
import rospy
from nav_msgs.msg import Odometry

class Shared(object):
    def __init__(self):
        self.yaw = None
        self.depth = None
        self.start_time = None
        self.absolute_pose = Odometry()

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
