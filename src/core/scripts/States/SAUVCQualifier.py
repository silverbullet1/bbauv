import rospy, smach, actionlib

from bbauv_msgs.msg import ControllerGoal

class Wait(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world

    def execute(self, userdata):
        rospy.sleep(self.world.config['qualifier_wait'])
        self.world.static_yaw = self.world.current_yaw
        print self.world.static_yaw 
        if self.world.static_yaw is not None:
            return 'pass'
        else:
            return 'fail'

class Dive(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world
        self.world.static_yaw = self.world.current_yaw

    def execute(self, userdata):
        self.world.enable_PID()
        self.world.lights.publish(9)
        self.world.lights.publish(9)
        self.world.lights.publish(5)
        goal = ControllerGoal(depth_setpoint=self.world.config['qualifier_depth'],
                sidemove_setpoint=0,
                heading_setpoint=self.world.static_yaw,
                forward_setpoint=0)
        rospy.loginfo("Waiting for locomotionServer before diving")
        self.world.locomotionServer.wait_for_server()
        rospy.loginfo("We are diving")
        r = self.world.locomotionServer.send_goal_and_wait(goal,
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
        ControllerGoal(depth_setpoint=self.world.config['qualifier_depth'],
                       sidemove_setpoint=0,
                       heading_setpoint=self.world.static_yaw,
                       forward_setpoint=self.world.config['qualifier_forward'])
        self.world.locomotionServer.wait_for_server()
        rospy.loginfo("Going forward by %f meters" %
                      (self.world.config['qualifier_forward']))
        r = self.world.locomotionServer.send_goal_and_wait(goal,
                                                   execute_timeout=rospy.Duration(150))
        if r == actionlib.GoalStatus.SUCCEEDED:
            goal = ControllerGoal(depth_setpoint=0, sidemove_setpoint=0,
                                    heading_setpoint=self.world.static_yaw,
                                    forward_setpoint=0)
            rrr = self.world.locomotionServer.send_goal_and_wait(goal)
            if rrr == actionlib.GoalStatus.SUCCEEDED:
                return 'pass'
        else:
            return 'fail'

class State(smach.State):
    transitions = {'pass' : 'pass', 'fail' : 'fail'}
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
        self.world.lights.publish(4)
        outcome = self.sm.execute()
        self.world.disable_PID()
        return outcome
