#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('core')
import rospy, smach, actionlib
import sys
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import core.cfg.mission_plannerConfig as Config
from bbauv_msgs.srv import set_controller
from bbauv_msgs.srv import vision_to_mission, mission_to_vision
from bbauv_msgs.srv import vision_to_missionResponse
from bbauv_msgs.msg import compass_data, depth, ControllerAction, controller
from nav_msgs.msg import Odometry
#from std_msgs.msg import Int8

class Interaction(object):
    def __init__(self):
        self.current_depth = None
        self.current_yaw = None
        self.current_pos = {'x': None, 'y': None}

        try:
            rospy.loginfo("Waiting for compass/depth")
            rospy.Subscriber("/euler", compass_data, lambda d:
                            setattr(self, 'current_yaw', d.yaw))
            rospy.Subscriber("/depth", depth, lambda d:
                            setattr(self, 'current_depth', d.depth))

            rospy.loginfo("Waiting for controller.")
            self.initController = rospy.ServiceProxy("/set_controller_srv",
                                                    set_controller)
            self.initController.wait_for_service()

            rospy.loginfo("Waiting for locomotionServer")
            self.controller = actionlib.SimpleActionClient(
                    "LocomotionServer", ControllerAction
                )
            self.controller.wait_for_server()

            rospy.loginfo("Waiting to populate ahrs8")
            while self.current_yaw is None:
                rospy.sleep(0.5)

            while self.current_depth is None:
                rospy.sleep(0.5)
            rospy.loginfo("Init done")

            rospy.Subscriber("/earth_odom", Odometry, self.DVLCallback)

            rospy.loginfo("Collecting earth_odom")
            while self.current_pos['x'] is None:
                rospy.sleep(0.5)

        except rospy.ServiceException as e:
            rospy.logerr("Exception starting mission critical things: %s"
                        % str(e))

    def DVLCallback(self, d):
        self.current_pos['x'] = d.pose.pose.position.x
        self.current_pos['y'] = d.pose.pose.position.y

    def enable_PID(self):
            self.initController(forward=True, sidemove=True,
                                roll=True, pitch=True, heading=True,
                                depth=True, navigation=False,topside=False)

    def disable_PID(self):
            self.initController(forward=False, sidemove=False,
                                roll=False, pitch=False, heading=False,
                                depth=False, navigation=False, topside=False)

    def startLaneServer(self):
        self.laneHeadingReturned = None
        self.laneHeadingGiven = None
        self.laneStatus = {'aborted' : False,
                           'active' : False,
                           'complete' : False,
                           'heading': None,
                           'numMarkers' : 0}
        self.laneServer = rospy.Service("/lane/vision_to_mission",
                                       vision_to_mission, self.laneCallback)

    def laneCallback(self, r):
        if r.task_complete_request:
            rospy.loginfo(str(r))
            self.laneStatus['complete'] = True
            self.laneStatus['heading'] = r.task_complete_ctrl.heading_setpoint
            return vision_to_missionResponse(search_response=True,
                                            task_complete_response=True,
                                            data=controller())
        if r.fail_request:
            self.laneStatus['aborted'] = True
            return vision_to_missionResponse(search_response=False,
                                             task_complete_response=False,
                                             data=controller())

    def startRGBServer(self):
        self.headingReturned = None
        self.headingGiven = None
        self.RGBStatus = {'aborted' : False,
                          'active': False,
                          'failed':False,
                          'done' : False}

        self.RGBServer = rospy.Service("/rgb/vision_to_mission",
                                      vision_to_mission, self.RGBCallback)

    def RGBCallback(self, r):
        if r.task_complete_request:
            self.RGBStatus['done'] = True
            return vision_to_missionResponse(search_response=False,
                                            task_complete_ctrl=controller(),
                                            task_complete_response=True)

        if r.fail_request:
            self.RGBStatus['failed'] = True
            return vision_to_missionResponse(search_response=False,
                                            task_complete_ctrl=controller(),
                                            task_complete_response=False)



class MissionPlanner(object):
    def __init__(self):
        rospy.init_node("mission_planner", anonymous=False)
        self.modules = {}
        self.loadedStates = {}
        self.interact = Interaction()
        self.helper = []
        self.missions = smach.StateMachine(outcomes=['pass', 'fail'])
        self.reconfigure_server = DynamicReconfigureServer(Config,
                                                           self.cCallback)
        self.reconfigure_client = DynamicReconfigureClient("/DVL")
    def cCallback(self, config, level):
        rospy.loginfo("Reconfigure callback")
        self.interact.config = config
        return config

    def load_state(self, name):
        name = "States.%s" % (name)
        try:
            __import__(name)
        except ImportError, e:
            rospy.error("Error importing state: %s" % (str(e)))
            return None
        module = sys.modules[name]
        cls = module.State
        self.loadedStates[name] = cls
        self.modules[name] = module
        self.helper.append(name)

    def add_missions(self):
        with self.missions:
            for k in self.helper:
                #trans = getattr(self.loadedStates[k], 'transitions')
                if((len(self.loadedStates.keys())) == 1):
                    trans = {'pass' : 'pass', 'fail' : 'fail'}
                else:
                    trans = getattr(self.loadedStates[k], 'transitions')
                smach.StateMachine.add(str(k).split('.')[1],
                                       self.loadedStates[k](self.interact),
                                       transitions=trans)

    def run(self):
        self.add_missions()
        rospy.loginfo("Resetting earth odometer")
        self.reconfigure_client.update_configuration({'zero_distance' : True})
        outcome = self.missions.execute()
        return outcome

if __name__ == "__main__":
    if len(sys.argv) <= 1:
        sys.stderr.write("Not enough arguments")
        exit(1)
    m = MissionPlanner()
    rospy.loginfo("Running states: %s", str(sys.argv[1:]))
    map(m.load_state, map(str, sys.argv[1:]))
    t = time.time()
    outcome = m.run()
    tt = time.time()
    rospy.loginfo("Time taken for %s: %f" % (str(sys.argv[1:]), tt - t))

