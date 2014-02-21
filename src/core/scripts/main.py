#!/usr/bin/env python

import roslib; roslib.load_manifest('core')
import rospy, smach
import sys

from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

class Interaction(object):
    def __init__(self):
        self.depth = 0
        self.yaw = 0

        try:
            self.ControllerSettings = rospy.ServiceProxy("/set_controller_srv",
                                                         set_controller)
            self.compassService = rospy.Subscriber("/euler", compass_data,
                                                   lambda d: setattr(self,
                                                                     'yaw',
                                                                     d.yaw))
            self.depthService = rospy.Subscriber("/depth", depth, lambda d:
                                                 setattr(self, 'depth', d))
        except rospy.ServiceException, e:
            rospy.logerr("Error subscribing to service: %s" % (str(e)))

    def enable_PID(self):
        self.ControllerSettings(forward=True, sidemove=True, heading=True,
                                depth=True, roll=False, topside=False,
                                navigation=False)

    def disable_PID(self):
        self.ControllerSettings(forward=False, sidemove=False, heading=False,
                                depth=False, roll=False, topside=False,
                                navigation=False)


class MissionPlanner(object):
    def __init__(self):
        rospy.init_node("mission_planner", anonymous=False)
        self.modules = {}
        self.loadedStates = {}
        self.interact = Interaction()
        self.missions = smach.StateMachine(outcomes=['pass', 'fail'])
    
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

    def add_missions(self):
        with self.missions:
            for k in self.loadedStates.keys():
                trans = getattr(self.loadedStates[k], 'transitions')
                smach.StateMachine.add(str(k),
                                       self.loadedStates[k](self.interact),
                                       transitions=trans)

    def run(self):
        self.add_missions()
        self.missions.execute()

if __name__ == "__main__":
    m = MissionPlanner()
    m.load_state('Qualifier')
    m.run()
