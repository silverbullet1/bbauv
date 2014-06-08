import rospy, smach
from bbauv_msgs.msg import controller
from bbauv_msgs.srv import mission_to_vision

class RGB(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world

    def execute(self, userdata):
        rospy.loginfo("Waiting for buoy service")
        self.world.RGBClient = rospy.ServiceProxy("/rgb/mission_to_vision",
                                                 mission_to_vision)
        self.world.RGBClient.wait_for_service()
        self.world.RGBClient(start_request=True,
                            abort_request=False,
                            start_ctrl=controller(
                                    depth_setpoint=2.5,
                                    heading_setpoint=
                                self.world.laneStatus['heading']
                                ))
        while not rospy.is_shutdown():
            if self.world.RGBStatus['done']:
                return 'pass'
            if self.world.laneStatus['aborted']:
                return 'fail'
            if self.world.laneStatus['failed']:
                return 'fail'

class State(smach.State):
    transitions = {'pass' : 'pass', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.world.startRGBServer()
        self.sm = smach.StateMachine(outcomes=self.outcomes)
        with self.sm:
            smach.StateMachine.add('RGB', RGB(self.world),
                                   transitions={'pass':'pass',
                                                'fail':'fail'})

    def execute(self, userdata):
        outcome = self.sm.execute()
        self.world.controller.cancel_all_goals()
        return outcome

