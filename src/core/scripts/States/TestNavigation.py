import rospy, smach
import numpy as np
from bbauv_msgs.msg import ControllerGoal

class Navigate(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world
        self.points = 4
        self.distance = 5

    def execute(self, userdata):
        self.world.enable_PID()
        yaw = self.world.current_yaw
        ex = self.distance * np.cos(np.radians(yaw))
        ey = self.distance * np.sin(np.radians(yaw))

        rospy.loginfo("Using yaw: %f, expected: (%f, %f)", yaw, ex, ey)

        goal = ControllerGoal(forward_setpoint=5, sidemove_setpoint=0,
                             heading_setpoint=yaw,
                             depth_setpoint=self.world.current_depth)
        self.world.controller.send_goal_and_wait(goal)
        errorx = ex - self.world.current_pos['x']
        errory = ey - self.world.current_pos['y']

        rospy.loginfo("Results: (%f, %f) Error x: %f, error y: %f",
                      self.world.current_pos['x'],
                      self.world.current_pos['y'],
                      errorx, errory)

        rospy.loginfo("Turning 90")

        yaw += 90

        ex = ex + self.distance * np.cos(np.radians(yaw))
        ey = ey + self.distance * np.sin(np.radians(yaw))

        rospy.loginfo("Using yaw: %f, expected: (%f, %f)", yaw, ex, ey)

        goal = ControllerGoal(forward_setpoint=0, sidemove_setpoint=0,
                             heading_setpoint=yaw,
                             depth_setpoint=self.world.current_depth)
        self.world.controller.send_goal_and_wait(goal)
        goal = ControllerGoal(forward_setpoint=5, sidemove_setpoint=0,
                             heading_setpoint=yaw,
                             depth_setpoint=self.world.current_depth)
        self.world.controller.send_goal_and_wait(goal)

        errorx = ex - self.world.current_pos['x']
        errory = ey - self.world.current_pos['y']

        rospy.loginfo("Results: (%f, %f) Error x: %f, error y: %f",
                      self.world.current_pos['x'],
                      self.world.current_pos['y'],
                      errorx, errory)

        rospy.loginfo("Turning 90")

        yaw += 90

        ex = ex + self.distance * np.cos(np.radians(yaw))
        ey = ey + self.distance * np.sin(np.radians(yaw))

        rospy.loginfo("Using yaw: %f, expected: (%f, %f)", yaw, ex, ey)

        goal = ControllerGoal(forward_setpoint=0, sidemove_setpoint=0,
                             heading_setpoint=yaw,
                             depth_setpoint=self.world.current_depth)
        self.world.controller.send_goal_and_wait(goal)
        goal = ControllerGoal(forward_setpoint=5, sidemove_setpoint=0,
                             heading_setpoint=yaw,
                             depth_setpoint=self.world.current_depth)
        self.world.controller.send_goal_and_wait(goal)

        errorx = ex - self.world.current_pos['x']
        errory = ey - self.world.current_pos['y']

        rospy.loginfo("Results: (%f, %f) Error x: %f, error y: %f",
                      self.world.current_pos['x'],
                      self.world.current_pos['y'],
                      errorx, errory)

        rospy.loginfo("Turning 90")

        yaw += 90

        ex = ex + self.distance * np.cos(np.radians(yaw))
        ey = ey + self.distance * np.sin(np.radians(yaw))

        rospy.loginfo("Using yaw: %f, expected: (%f, %f)", yaw, ex, ey)

        goal = ControllerGoal(forward_setpoint=0, sidemove_setpoint=0,
                             heading_setpoint=yaw,
                             depth_setpoint=self.world.current_depth)
        self.world.controller.send_goal_and_wait(goal)
        goal = ControllerGoal(forward_setpoint=5, sidemove_setpoint=0,
                             heading_setpoint=yaw,
                             depth_setpoint=self.world.current_depth)
        self.world.controller.send_goal_and_wait(goal)

        errorx = ex - self.world.current_pos['x']
        errory = ey - self.world.current_pos['y']

        rospy.loginfo("Results: (%f, %f) Error x: %f, error y: %f",
                      self.world.current_pos['x'],
                      self.world.current_pos['y'],
                      errorx, errory)

        if errorx > 0.2 or errory > 0.2:
            return 'fail'
        return 'pass'

class State(smach.State):
    transitions = {'pass': 'pass', 'fail': 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.sm = smach.StateMachine(outcomes=self.outcomes)
        with self.sm:
            smach.StateMachine.add('NAV', Navigate(self.world),
                                   transitions={'pass':'pass',
                                                'fail':'fail'})

    def execute(self, userdata):
        outcome = self.sm.execute()
        self.world.controller.cancel_all_goals()
        self.world.disable_PID()
        return outcome
