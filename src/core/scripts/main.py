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
from std_msgs.msg import Int8

class Interaction(object):
    def __init__(self):
        self.current_depth = None
        self.current_yaw   = None
        self.static_yaw = None
        self.current_pos = {'x' : 0, 'y' : 0}

        self.grace = 0

        self.linefollowerActive = False
        self.linefollowerDone   = False
        self.bucketActive       = False
        self.bucketDone         = False
        self.flareActive        = False
        self.flareDone          = False

        self.acousticsActive    = False
        self.acousticsDone      = False

        self.LinefollowerFailed = False
        self.bucketFailed       = False
        self.flareFailed        = False
        self.acousticsFailed    = False

        """
        need to subscribe to ahrs8, controller, depth
        """
        try:
            rospy.Subscriber("/euler", compass_data, lambda d: 
                             setattr(self, 'current_yaw', d.yaw))
            rospy.loginfo("Subscribed to AHRS8")
            rospy.Subscriber("/depth", depth, lambda d: 
                             setattr(self, 'current_depth', d.depth))
            rospy.loginfo("Subscribed to depth")
            
            self.setController = rospy.ServiceProxy("/set_controller_srv",
                                                 set_controller)

            rospy.loginfo("Waiting for PID Controller")
            self.setController.wait_for_service()
            rospy.loginfo("Got PID Controller")
            self.locomotionServer =\
            actionlib.SimpleActionClient("/LocomotionServer", ControllerAction)
            rospy.loginfo("Waiting for locomotion server")
            self.locomotionServer.wait_for_server()
            rospy.loginfo("Got locomotion server")
            rospy.Subscriber("/earth_odom", Odometry, self.DVLCallback)
        except rospy.ServiceException, e:
            rospy.logerr("Error subscribing to critical services, exiting")
            exit(1)

        rospy.loginfo("Waiting for compass data to get populated")
        while self.current_yaw is None:
            rospy.sleep(rospy.Duration(0.5))
        rospy.loginfo("Compass data populated")

        rospy.loginfo("Waiting for DVL data to be populated")
        while self.current_pos['x'] or self.current_pos['y'] is None:
            rospy.sleep(rospy.Duration(0.5))
        rospy.loginfo("DVL populated")

        rospy.loginfo("Starting mission specific services")

        try:
            self.linefollowerServer     =\
            rospy.Service("/linefollower/vision_to_mission", vision_to_mission,
                         self.linefollowerServerCallback)
            self.linefollowerService    =\
            rospy.ServiceProxy("/linefollower/mission_to_vision",
                               mission_to_vision)
            self.bucketServer           =\
            rospy.Service("/bucket/vision_to_mission", vision_to_mission,
                         self.bucketServerCallback)
            self.bucketService          =\
            rospy.ServiceProxy("/bucket/mission_to_vision", mission_to_vision)
            self.flareServer            =\
            rospy.Service("/flare/vision_to_mission", vision_to_mission,
                          self.flareServerCallback)
            self.flareService           =\
            rospy.ServiceProxy("/flare/mission_to_vision", mission_to_vision)
            self.acousticService        =\
            rospy.ServiceProxy("/acoustic/mission_to_vision", mission_to_vision)
            self.acousticServer         =\
            rospy.Service("/acoustic/vision_to_mission", vision_to_mission,
                          self.acousticServerCallback)
        except rospy.ServiceException, e:
            rospy.logerr("Error creating task specific services: %s" % (str(e)))

        try:
            self.lights = rospy.Publisher("/led_strips", Int8)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot proc lights.")

    def DVLCallback(self, data):
        self.current_pos['x'] = data.pose.pose.position.x
        self.current_pos['y'] = data.pose.pose.position.y

    def linefollowerServerCallback(self, req):
        if req.fail_request:
            rospy.loginfo("Linefollower failed.")
            self.LinefollowerFailed = True
            return vision_to_missionResponse(search_response=False,
                                                task_complete_response=False,
                                                data=controller())

    def bucketServerCallback(self, req):
        if req.fail_request:
            rospy.loginfo("Bucket node failed")
            self.bucketFailed = True
            return vision_to_missionResponse(search_response=False,
                                                task_complete_response=False,
                                                data=controller())
        if req.search_request:
            self.linefollowerDone = True
            rospy.loginfo("Aborting linefollower because we found the bucket")
            r = self.linefollowerService(abort_request=True, start_request=False,
                                     start_ctrl=controller())
            self.linefollower_last_heading = r.data.heading_setpoint
            rospy.loginfo("Giving bucket the linefollower heading of %f" %
                          (self.linefollower_last_heading))
            return vision_to_missionResponse(search_response=True,
                    task_complete_response=False,
                    data=controller(heading_setpoint=self.linefollower_last_heading))
        if req.task_complete_request:
            self.bucketDone = True
            return vision_to_missionResponse(search_response=False,
                                             task_complete_response=True,
                                             data=controller())

    def flareServerCallback(self, data):
        if data.task_complete_request:
            self.flareDone = True
            return vision_to_missionResponse(
                search_response=False, task_complete_response=True,
                data=controller())
        if data.fail_request:
            self.flareFailed = True
            rospy.loginfo("FLARE REPORTED FAILURE")
            return vision_to_missionResponse(
                search_response=False, task_complete_response=False,
                data=controller())



    def enable_PID(self):
        self.setController(forward=True, sidemove=True, roll=True,
                            pitch=True, heading=True, depth=True,
                            navigation=False, topside=False)

    def disable_PID(self):
        self.setController(forward=False, sidemove=False, roll=False,
                            pitch=False, heading=False, depth=False,
                            navigation=False, topside=False)

    def activateLinefollower(self):
        if not self.static_yaw:
            self.starting_yaw = self.current_yaw
        rospy.loginfo("Activating linefollower node")
        r = self.linefollowerService(start_request=True,
                start_ctrl=controller(depth_setpoint=self.config['linefollower_depth'],
                            heading_setpoint=self.static_yaw),
                            abort_request=False)
        rospy.loginfo("Reply from linefollower: %s" % (str(r)))
        self.linefollowerActive = True

    def activateBucket(self):
        rospy.loginfo("Activating bucketdetector node")
        r = self.bucketService(start_request=True,
                               start_ctrl=controller(depth_setpoint=self.config[
                               'bucket_depth']))
        rospy.loginfo("Reply from bucketdetector: %s" % (str(r)))
        self.bucketActive = True

    def acousticServerCallback(self, req):
        if req.task_complete_request:
            self.acousticsDone = True
            #redo
            return vision_to_missionResponse(True, True, controller())
        return vision_to_missionResponse(True, True, controller())


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

