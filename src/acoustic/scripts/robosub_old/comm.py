#!/usr/bin/env python
import rospy 
import roslib
import actionlib
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from acoustic_stream import AcousticStream
import signal

#GLOBAL
defaultDepth = 1.0
class Comm:

    def __init__(self):

        #Init values
        self.depth= 0
        self.heading = 0
        self.doa = 0
        self.elevation= 0
        self.initDuration = 3
        self.e_low = 30
        self.count = 0
        self.step_size = 4.0

        self.isAlone = True
        self.isKilled = False
        self.isDone = False

        self.bound = {'low':140, 'high':240}


    #Init subs
    def adjustStep(self):
        if(60 < self.elevation < 70):
            self.step_size = 2.0
        elif(40 < self.elevation < 50):
            self.step_size = 1.0

    def kickstart(self):
    # Communicate with motion control server
        self.motionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                         ControllerAction)
        try:
            rospy.loginfo("Waiting for LocomotionServer...")
            self.motionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.loginfo("LocomotionServer timeout!")

        setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        setServer(forward=True, sidemove=True, heading=True, depth=True,
                  pitch=True, roll=True, topside=False, navigation=False)
        if not self.isAlone:
            # Initialize mission planner communication server and client
            rospy.loginfo("Starting /lane/mission_to_vision")
            self.comServer = rospy.Service("/mission_to_acoustic",
                                           mission_to_vision,
                                           self.handleSrv)
            rospy.loginfo("Waiting for vision to mission service")
            self.toMission = rospy.ServiceProxy("/acoustic_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service(timeout=60)

        self.compass_sub = rospy.Subscriber("/euler", compass_data, self.compass_callback)
        self.depth_sub = rospy.Subscriber("/depth", depth, self.depth_callback)
        self.acoustic_sub = rospy.Subscriber("/acoustic/pingu", pingu, self.acoustic_callback)

    def overShotPinger(self, angle):
        return self.bound['low'] < angle < self.bound['high']
    
    def compass_callback(self, data):
        self.heading = data.yaw
        
    def depth_callback(self, data):
        self.depth = data.depth

    def acoustic_callback(self, data):
        self.doa = data.doa
        self.elevation = data.elevation

    def sendMovement(self,forward=0.0, sidemove=0.0, turn=None, depth=defaultDepth, absolute=False, wait=True):
        if turn is None:
            turn = self.heading
            goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        else:
            if absolute:
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
            else:
                turn = (turn+self.heading)%360 
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        rospy.loginfo("Turn received: " + str(turn))
        self.motionClient.send_goal(goal)
        if wait:
            self.motionClient.wait_for_result()
        else:
            self.motionClient.wait_for_result(timeout=rospy.Duration(2.0))

    def handleSrv(self, req):
        if req.start_request:
            rospy.loginfo("Received Start Request")
            self.isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.heading))
        elif req.abort_request:
            rospy.loginfo("Received Abort Request")
            self.sendMovement(forward=0.0, sidemove=0.0)
            self.isAborted = True
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.heading))

if __name__ == "__main__":
    testComm = Comm()
