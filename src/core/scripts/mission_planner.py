#!/usr/bin/python

import roslib; roslib.load_manifest('core')
import rospy, actionlib
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

import smach

"""
Mission plan for SAUV 2014
"""

class Interaction(object):
    """
    this is the entry point for interaction with outside resources to the state
    machine. it contains all the service proxies and servers, meant to be passed
    to all the states. gets rid of global variables.
    the boolean values are checked in states
    """
    def __init__(self):
        self.linefollower_done = False
        self.flare_found = False
        self.flare_ended = False
        self.vision_serviceproxy = rospy.ServiceProxy('/linefollower/mission_to_vision',
                                                 mission_to_vision)
        self.vision_serviceserver = rospy.Service('/linefollower/vision_to_mission', 
                                                  vision_to_mission,
                                                  self.linefollower)

        self.flare_serviceproxy = rospy.ServiceProxy('/flare/mission_to_flare',
                                                     mission_to_vision)
        self.flare_serviceserver = rospy.Service('/flare/flare_to_mission',
                                                 vision_to_mission,
                                                 self.flareState)

    def flareState(self, req):
        if req.search_request:
            self.flare_found = True
        if req.task_complete_request:
            self.flare_ended = True
    def linefollower(self, req):
        rospy.loginfo(req.task_complete_request)
        if req.task_complete_request:
            self.linefollower_done = True

class InitialState(smach.State):
    """
    Initial state of the mission controller.
    We dive 0.1m and start the LineFollower node.
    """
    def __init__(self):
        self.isDone = False
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        self.actionClient = actionlib.SimpleActionClient('LocomotionServer',
                                                    ControllerAction)
        self.goal = ControllerGoal(depth_setpoint=0.0)

    def done(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.isDone = True
        elif status == actionlib.GoalStatus.PREEMPTED:
            self.execute()

    def execute(self, userdata):
        self.actionClient.wait_for_server()
        while not rospy.is_shutdown():
            if self.isDone:
                return 'initialized'
            else:
                self.actionClient.send_goal(self.goal, self.done)
                self.actionClient.wait_for_result()

class Gate(smach.State):
    """
    part that we're supposed to go under the gate
    also known as line follower and vision
    """
    def __init__(self, world):
        self.world = world
        smach.State.__init__(self, outcomes=['gate_passed', 'gate_failed'])

    def execute(self, userdata):
        ##call linefollower here
        rospy.wait_for_service('/linefollower/mission_to_vision')
        self.world.vision_serviceproxy(start_request=True,
                                       start_ctrl=controller(depth_setpoint=0.1),
                                       abort_request=False)
        while not rospy.is_shutdown():
            if self.world.linefollower_done:
                return 'gate_passed'


class Bucket(smach.State):
    """
    dropping ball in bucket
    must be activated and start and will send a request to the mission
    controller when linefollower detects a curve of more than 45degs
    then must be handed control to
    """
    def __init__(self, world):
        self.world = world
        smach.State.__init__(self, outcomes=['ball_dropped', 'ball_failed'])

    def execute(self, userdata):
        self.world.vision_serviceproxy(abort_request=True)
        if self.world.linefollower_done:
            return 'ball_dropped'

class Flare(smach.State):
    """
    supposed to drop a ball off a flare
    the detector should be running after the bucket state and will request
    control
    """
    def __init__(self, world):
        self.world = world
        smach.State.__init__(self, outcomes=['flare_done', 'flare_failed'])

    def execute(self, userdata):
        return 'flare_done'

class Acoustics(smach.State):
    """
    last state
    also fallback state
    """
    def __init__(self, world):
        self.world = world
        smach.State.__init__(self, outcomes=['acoustics_done',
                                             'acoustics_notfound',
                                             'acoustics_failed'])

    def execute(self, userdata):
        return 'acoustics_done'

class Mission_planner(object):
    def __init__(self):
        rospy.init_node('Mission_planner', anonymous=False)
        self.missions = smach.StateMachine(outcomes=['mission_complete', 'gg'])
        self.interact = Interaction()


    def add_missions(self):
        with self.missions:
            smach.StateMachine.add('INIT', InitialState(), transitions={
                'initialized' : 'GATE',
                'failed' : 'ACST'
            })
            smach.StateMachine.add('ACST', Acoustics(self.interact), transitions={
                'acoustics_done' : 'mission_complete',
                'acoustics_notfound' : 'gg',
                'acoustics_failed' : 'gg'
            })
            smach.StateMachine.add('GATE', Gate(self.interact), transitions={
                'gate_passed' : 'BUCKET',
                'gate_failed' : 'ACST'
            })
            smach.StateMachine.add('BUCKET', Bucket(self.interact), transitions={
                'ball_dropped' : 'FLARE',
                'ball_failed' : 'ACST'
            })
            smach.StateMachine.add('FLARE', Flare(self.interact), transitions={
                'flare_done' : 'ACST',
                'flare_failed' : 'ACST'
            })

    def run(self):
        self.add_missions()
        try:
            outcome = self.missions.execute()
            if outcome == 'mission_success':
                rospy.loginfo('Mission done')
            else:
                rospy.loginfo('Atleast we tried')
            rospy.spin()
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    sm = Mission_planner()
    sm.run()
