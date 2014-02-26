import rospy, smach, actionlib

from bbauv_msgs.msg import ControllerGoal

class Wait(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world

    def execute(self, userdata):
        rospy.sleep(self.world.config['qualifier_wait'])
        if self.world.static_yaw is not None:
            return 'pass'
        else:
            return 'fail'

class Dive(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world
        self.world.static_yaw = self.world.yaw

    def execute(self, userdata):
        goal = ControllerGoal(depth_setpoint=self.world.config['sauvc_depth'],
                sidemove_setpoint=0,
                heading_setpoint=self.world.static_yaw,
                forward_setpoint=0)
        rospy.loginfo("Waiting for actionServer before diving")
        self.world.actionServer.wait_for_server()
        rospy.loginfo("We are diving")
        r = self.world.actionServer.send_goal_and_wait(goal,
                                                   execute_timeout=rospy.Duration(60))
        if r == actionlib.GoalStatus.SUCCEEDED:
            return 'pass'
        else:
            return 'fail'

class Forward(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world

    def execute(self, userdata):
        goal =\
        ControllerGoal(depth_setpoint=self.world.config['sauvc_depth'],
                       sidemove_setpoint=0,
                       heading_setpoint=self.world.static_yaw,
                       forward_setpoint=self.world.config['sauvc_forward'])
        self.world.actionServer.wait_for_server()
        r = self.world.actionServer.send_goal_and_wait(goal,
                                                   execute_timeout=rospy.Duration(300))
        if r == actionlib.GoalStatus.SUCCEEDED:
            return 'pass'
        else:
            return 'fail'

class State(smach.State):
    transitions = {'pass' : 'SAUVCLinefollower', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.sm = smach.StateMachine(outcomes=self.outcomes)
        with self.sm:
            smach.StateMachine.add('WAIT', Wait(self.world), transitions={'pass' : 'DIVE',
                                                                          'fail' : 'fail'})
            smach.StateMachine.add('DIVE', Dive(self.world), transitions={'pass' : 'FWD',
                                                                'fail' : 'fail'})
            smach.StateMachine.add('FWD', Forward(self.world), transitions={'pass' :
                                                                  'pass',
                                                                  'fail': 'fail'})


    def execute(self, userdata):
        self.world.enable_PID()
        outcome = self.sm.execute()
        self.world.actionServer.cancel_all_goals()
        return outcome

