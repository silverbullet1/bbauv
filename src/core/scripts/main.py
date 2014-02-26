#!/usr/bin/env python

import roslib; roslib.load_manifest('core')
import rospy, smach, actionlib
import sys
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import core.cfg.mission_plannerConfig as Config
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

class Interaction(object):
    def __init__(self):
        self.depth = None
        self.yaw = None
        self.static_yaw = None

        try:
            self.ControllerSettings = rospy.ServiceProxy("/set_controller_srv",
                                                         set_controller)
            self.compassService = rospy.Subscriber("/euler", compass_data,
                                                   lambda d: setattr(self,
                                                                     'yaw',
                                                                     d.yaw))
            self.actionServer = actionlib.SimpleActionClient("LocomotionServer",
                                                             ControllerAction)
            rospy.loginfo("Waiting for compass to get populated")
            while self.yaw is None:
                rospy.sleep(rospy.Duration(1))
            rospy.loginfo("done waiting for compass")
           # self.depthService = rospy.Subscriber("/depth", depth, lambda d:
           #                                      setattr(self, 'depth',
           #                                              d.depth.depth))
        except rospy.ServiceException, e:
            rospy.logerr("Error subscribing to service: %s" % (str(e)))

    def enable_PID(self):
        self.ControllerSettings(forward=True, sidemove=True, heading=True,
                                depth=True, roll=False, topside=False,
                                pitch=True,
                                navigation=False)

    def disable_PID(self):
        self.ControllerSettings(forward=False, sidemove=False, heading=False,
                                depth=False, roll=False, topside=False,
                                pitch=False,
                                navigation=False)


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
        self.reconfigure_client = DynamicReconfigureClient("/earth_odom")
    def cCallback(self, config, level):
        rospy.loginfo("Reconfigured")
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
        self.reconfigure_client.update_configuration({'zero_distance' : True})
        self.missions.execute()

if __name__ == "__main__":
    m = MissionPlanner()
    #m.load_state('SAUVCQualifier')
    m.load_state('SAUVCInitial')
    m.load_state('SAUVCLinefollower')
    #m.load_state('SAUVCInitial')
    m.run()
