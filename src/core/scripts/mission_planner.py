#!/usr/bin/python

import roslib; roslib.load_manifest('core')
import rospy, actionlib
from std_msgs.msg import String
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

import smach

"""
Mission plan for SAUV 2014
"""


class InitialState(smach.State):
    def __init__(self):
        self.isDone = False
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        self.actionClient = actionlib.SimpleActionClient('LocomotionServer',
                                                    ControllerAction)
        self.goal = ControllerGoal(depth_setpoint=0.1)

    def done(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.isDone = True
        elif status == actionlib.GoalStatus.PREEMPTED:
            self.execute()

    def execute(self, userdata):
        return 'initialized' # temporary pass through to test linefollower
        status = 'mission_planer.init.start'
        self.actionClient.wait_for_server()
        #self.actionClient.send_goal(self.goal, self.done)
        #self.actionClient.wait_for_result()
        while not rospy.is_shutdown():
            if self.isDone:
                return 'initialized'
            else:
                self.actionClient.send_goal(self.goal, self.done)
                self.actionClient.wait_for_result()

class Gate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gate_passed', 'gate_failed'])

    def execute(self, userdata):
        ##call linefollower here
        rospy.wait_for_service('linefollower_service')
        service = rospy.ServiceProxy('linefollower_service',
                                     mission_linefollower)
        resp = service(True, True)
        rospy.loginfo(resp.start_response)
        rospy.loginfo(resp.abort_response)
        return 'gate_passed'

class Bucket(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ball_dropped', 'ball_failed'])

    def execute(self, userdata):
        return 'ball_dropped'

class Flare(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flare_done', 'flare_failed'])

    def execute(self, userdata):
        return 'flare_done'

class Acoustics(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['acoustics_done'])

    def execute(self, userdata):
        return 'acoustics_done'

class Mission_planner(object):
    def __init__(self):
        rospy.init_node('Mission_planner', anonymous=False)
        self.missions = smach.StateMachine(outcomes=['mission_complete'])

    def add_missions(self):
        with self.missions:
            smach.StateMachine.add('INIT', InitialState(), transitions={
                'initialized' : 'GATE',
                'failed' : 'mission_complete'
            })
            smach.StateMachine.add('ACST', Acoustics(), transitions={
                'acoustics_done' : 'mission_complete'
            })
            smach.StateMachine.add('GATE', Gate(), transitions={
                'gate_passed' : 'BUCKET',
                'gate_failed' : 'ACST'
            })
            smach.StateMachine.add('BUCKET', Bucket(), transitions={
                'ball_dropped' : 'FLARE',
                'ball_failed' : 'ACST'
            })
            smach.StateMachine.add('FLARE', Flare(), transitions={
                'flare_done' : 'ACST',
                'flare_failed' : 'ACST'
            })

    def run(self):
        self.add_missions()
        try:
            outcome = self.missions.execute()
            rospy.spin()
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    sm = Mission_planner()
    sm.run()
