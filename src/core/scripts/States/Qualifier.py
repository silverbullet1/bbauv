import rospy, smach, actionlib

from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

class State(smach.State):
    transitions = {'pass' : 'pass', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        self.timer = rospy.Timer(rospy.Duration(120, self.fail, oneshot=True))
        self.world = world
        self.doneDiving = False
        self.done = False
        self.actionClient = actionlib.SimpleActionClient('LocomotionServer',
                                                    ControllerAction)
        smach.State.__init__(self, outcomes=self.outcomes)

    def forward(self, fwd=10.0):
        goal = ControllerGoal(depth_setpoint=self.world.depth,
                              sidemove_setpoint=0.0,
                              heading_setpoint=self.world.static_yaw,
                              forward_setpoint=fwd)
        rospy.loginfo("waiting for actionlib server before moving forward")
        self.actionClient.wait_for_server()
        outcome = self.actionClient.send_goal_and_wait(goal,
                                             execute_timeout=rospy.Duration(30),
                                             preempt_timeout=rospy.Duration(10))
        if outcome == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Forward succesfull, terminating.")
            return True
        else:
            return False

    def dive(self, depth=0.2):
        #self.world.depth = depth
        rospy.sleep(rospy.Duration(1))
        self.world.static_yaw = self.world.yaw
        goal = ControllerGoal(depth_setpoint=depth,
                                   heading_setpoint=self.world.static_yaw)
        rospy.loginfo("Waiting for actionlib before diving")
        outcome = self.actionClient.send_goal_and_wait(goal,
                                             execute_timeout=rospy.Duration(30),
                                             preempt_timeout=rospy.Duration(10))
        if outcome == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Dive successfull")
            return self.forward()
        return False

    def fail(self):
        raise NotImplementedError

    def execute(self, userdata):
        self.world.enable_PID()
        if self.dive():
            #self.world.disable_PID()
            return 'pass'
        else:
            self.world.disable_PID()
            return 'fail'
