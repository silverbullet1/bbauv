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
        self.heading = None

        """
        THESE VARIABLES ARE INITIAL STATES BUT WILL GET OVEWRITTEN BY THE SAME
        SHIT IF EVERYTHING GOES RIGHT
        THERE ARE DEFAULTS IN PLACE FOR SAFETY
        """
        self.depth = None
        self.forward = None

        rospy.loginfo("Initializing mission planner interaction module")

        try:
            self.ControllerSettings = rospy.ServiceProxy("/set_controller_srv",
                                                        set_controller)
            rospy.loginfo("Waiting for Controller Service")
            self.ControllerSettings.wait_for_service()
            rospy.loginfo("Got Controller Service, asking for forward, heading, \
                        depth control and pitch")
            self.ControllerSettings(forward=True, sidemove=False, heading=True,
                                    depth=True, roll=False, topside=False,
                                    navigation=False)
        except rospy.ServiceException, e:
            rospy.logerr("Error while subscribing to the Controller service: %s"
                         % (str(e)))
        self.initCompass()

    def initCompass(self):
        rospy.loginfo("Subscribing to the compass service")
        try:
            self.compassService = rospy.Subscriber("/euler", compass_data,
                                                   self.compassCallback)
        except rospy.ServiceException, e:
            rospy.logerr("Error while subscribing to the compass service: %s" %
                        (str(e)))

    def compassCallback(self, data):
        self.heading = data


class InitialState(smach.State):
    """
    Initial state of the mission controller.
    We dive 0.1m and start the LineFollower node.
    """
    def __init__(self, world):
        self.world = world
        self.DoneDiving = False
        self.DoneMoving = False
        smach.State.__init__(self, outcomes=['initialized', 'failed'])

    def dive(self, depth=0.1):
        self.world.depth = depth
        rospy.loginfo("Subscribing to the LocomotionServer to dive.")
        self.actionClient = actionlib.SimpleActionClient('LocomotionServer',
                                                         ControllerAction)
        self.goal = ControllerGoal(depth_setpoint=depth,
                                   heading_setpoint=self.world.heading)
        rospy.loginfo("Waiting for Actionlib")
        self.actionClient.wait_for_server()
        rospy.loginfo("Got Actionlib server, sending goal")
        self.actionClient.send_goal(self.goal, self.diveCallback)
        self.actionClient.wait_for_result()

    def diveCallback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Dive successfull")
            self.DoneDiving = True
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Diving preempted by actionlib server, waiting 10\
                          seconds")
            try:
                rospy.sleep(rospy.Duration(10))
            except rospy.ROSInterruptException, e:
                rospy.logerr("Sleep interrupted: %s" % str(e))
            self.dive() #may need to unsub from the actionlib server first
        else:
            rospy.loginfo("Unknown status caught: %s" % (str(status)))

    def goForward(self, distance=0.7):
        """
        we assume that this is only called when the dive is complete
        """
        self.goal = ControllerGoal(depth_setpoint=self.world.depth,
                                   heading_setpoint=self.world.heading,
                                   forward_setpoint=distance)
        rospy.loginfo("Waiting for Actionlib before moving forward")
        self.actionClient.wait_for_server()
        rospy.loginfo("Got actionlib server, sending goal to move forward")
        self.actionClient.send_goal(self.goal, self.forwardCallback)
        self.actionClient.wait_for_result()

    def forwardCallback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("We have successfully moved ahead")
            self.DoneMoving = True
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Moving forward preempted by actionlib, waiting 10\
                          seconds")
            try:
                rospy.sleep(rospy.Duration(10))
            except rospy.ROSInterruptException, e:
                rospy.logerr("Sleep interrupted: %s" % (str(e)))
            self.goForward()

    def done(self):
        #self.actionClient.unsubscribe()
        #there does not seem to be an unsubscribe method
        #raise NotImplementedException
        pass

    def execute(self, userdata):
        rospy.loginfo("Executing INIT state")
        while not rospy.is_shutdown():
            if self.DoneDiving and self.DoneMoving:
                rospy.loginfo("Done diving and moving forward. End of state.")
                self.done()
                return 'initialized'
            if not self.DoneDiving and not self.DoneMoving:
                self.dive(self.world.depth)
            if self.DoneDiving and not self.DoneMoving:
                self.goForward(0)

class Gate(smach.State):
    """
    part that we're supposed to go under the gate
    also known as line follower and vision
    """
    def __init__(self, world):
        self.world = world
        self.visionDone = False
        self.visionActive = False
        smach.State.__init__(self, outcomes=['gate_passed', 'gate_failed'])

    def activateVisionNode(self):
        rospy.loginfo("Trying to activate Vision node")
        try:
            self.visionService =\
            rospy.ServiceProxy("/linefollower/mission_to_vision",
                               mission_to_vision)
            rospy.loginfo("Waiting for vision node service")
            self.visionService.wait_for_service()
            response = self.visionService(start_request=True,
                               start_ctrl=controller(depth_setpoint=self.world.depth,
                                                     heading_setpoint=self.world.heading),
                               abort_request=False)
            if response.start_response:
                self.visionActive = True
                rospy.loginfo("Vision node replied TRUE, activated")
        except rospy.ServiceException, e:
            rospy.logerr("Service exception thrown: %s" % (str(e)))

        rospy.loginfo("Starting server for vision node to report to")
        try:
            self.visionServer = rospy.Service("/linefollower/vision_to_mission",
                                              vision_to_mission,
                                              self.visionCallback)
        except rospy.ServiceException, e:
            rospy.loginfo("Service exception thrown: %s" % (str(e)))

    def visionCallback(self, req):
        if req.task_complete_request:
            self.visionDone = True
        elif req.search_request and not self.visionActive:
            self.activateVisionNode()

    def shutdownVision(self):
        rospy.loginfo("Shutting down vision node if active.")
        if self.visionActive:
            self.visionService(start_request=False, start_ctrl=controller(),
                               abort_request=True)
            rospy.loginfo("Shutdown request sent")

    def execute(self, userdata):
        if not self.visionActive:
            self.activateVisionNode()
        while not rospy.is_shutdown():
            if self.visionDone:
                self.shutdownVision()
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
            smach.StateMachine.add('INIT', InitialState(self.interact), transitions={
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
