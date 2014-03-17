import rospy, smach
from bbauv_msgs.msg import controller

class State(smach.State):
    transitions = {'pass' : 'pass', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world

    def execute(self, userdata):
        rospy.loginfo("Waiting for flare service")
        self.world.flareService.wait_for_service()
        rospy.loginfo("Got flare service")
        self.world.flareService(
            start_request=True, abort_request=False,
            start_ctrl=controller(depth_setpoint=self.world.config['sauvc_depth'],
                                  heading_setpoint=self.world.current_yaw)
            )
        while not rospy.is_shutdown():
            if self.world.flareDone:
                return 'pass'
            if self.world.flareFailed:
                return 'fail'

