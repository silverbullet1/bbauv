import rospy, smach
from bbauv_msgs.msg import controller

class State(smach.State):
    transitions = {'pass' : 'pass', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        
    def execute(self, userdata):
        rospy.loginfo("Waiting for acoustics service")
        self.world.acousticService.wait_for_service()
        rospy.loginfo("Got acoustics service, executing")
        self.world.acousticService(
            start_request=True, abort_request=False,
                start_ctrl=controller())
        while not rospy.is_shutdown():
            if self.world.acousticsDone:
                return 'pass'
