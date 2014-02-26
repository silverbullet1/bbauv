import rospy, smach, actionlib

from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

class State(smach.State):
    outcomes = ['pass', 'fail']
    transitions = {'pass' : 'pass', 'fail' : 'fail'}

    def __init__(self, world):
        self.visionDone = False
        self.bucketDone = False
        self.world = world
        self.lineFollowerService =\
        rospy.ServiceProxy("/linefollower/mission_to_vision", mission_to_vision)
        self.missionService = rospy.Service("/linefollower/vision_to_mission",
                                            vision_to_mission,
                                            self.visionCallback)
        self.bucketService = rospy.Service("/bucket/vision_to_mission",
                                           vision_to_mission,
                                           self.bucketCallback)
        self.bucketdicks = rospy.ServiceProxy("/bucket/mission_to_vision",
                                              mission_to_vision)
        smach.State.__init__(self, outcomes=self.outcomes)

    def visionCallback(self, req):
        if req.task_complete_request:
            self.visionDone = True
            return vision_to_missionResponse(True, True)
        if req.search_request:
            self.visionDone = False
            self.activateVisionNode()
            return vision_to_missionResponse(True, True)
        if req.fail_request:
            self.visionDone = True
            return vision_to_missionResponse(True, True)

    def bucketCallback(self, req):
        if req.search_request:
            self.bucketDone = True


    def activateVisionNode(self):
        self.lineFollowerService.wait_for_service(timeout=10)
        r = self.lineFollowerService(start_request=True,
                                     start_ctrl=controller(depth_setpoint=0.1,
                                                          heading_setpoint=\
                                                          self.world.yaw),
                                     abort_request=False)
        rospy.loginfo(str(r))
        self.bucketdicks.wait_for_service(timeout=10)
        r = self.bucketdicks(start_request=True,
                                     start_ctrl=controller(depth_setpoint=0.2,
                                                          heading_setpoint=\
                                                          self.world.yaw),
                             abort_request = False)


    def execute(self, userdata):
        if not self.visionDone:
            self.activateVisionNode()
        while not rospy.is_shutdown():
            if self.visionDone and self.bucketDone:
                return 'pass'
        return 'fail'

