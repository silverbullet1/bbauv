import rospy, smach

from bbauv_msgs.msg import ControllerGoal

class TestGate(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world

    def execute(self, userdata):
        rospy.loginfo("Waiting 30 seconds")
        rospy.sleep(30)
        rospy.loginfo("Init PID")
        yaw = self.world.current_yaw;
        rospy.loginfo("Using static yaw of %s degrees" % str(yaw))
        self.world.enable_PID()
        fgoal = ControllerGoal(depth_setpoint=0.3,
                              sidemove_setpoint=0.0,
                              heading_setpoint=yaw,
                              forward_setpoint=11)

        dgoal = ControllerGoal(depth_setpoint=0.3, sidemove_setpoint=0,
                               heading_setpoint=yaw, forward_setpoint=0)

        rospy.loginfo("Diving")
        self.world.controller.wait_for_server()
        self.world.controller.send_goal_and_wait(dgoal,
                                                execute_timeout=rospy.Duration
                                                 (30))

        rospy.loginfo("Firing forward goal")
        self.world.controller.wait_for_server()
        self.world.controller.send_goal_and_wait(fgoal,
                                                execute_timeout=rospy.Duration
                                                (60))

        return 'pass'

class State(smach.State):
    transitions = {'pass': 'Lane', 'fail': 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.sm = smach.StateMachine(outcomes=self.outcomes)
        with self.sm:
            smach.StateMachine.add('LOL', TestGate(self.world),
                                   transitions={'pass':'pass',
                                                'fail':'fail'})

    def execute(self, userdata):
        outcome = self.sm.execute()
        self.world.controller.cancel_all_goals()
        self.world.disable_PID()
        return outcome
