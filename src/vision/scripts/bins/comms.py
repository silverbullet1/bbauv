import rospy
from dynamic_reconfigure.server import Server as DynServer

from bbauv_msgs.msg import manipulator, controller
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission

from utils.config import binsConfig as Config

from vision import BinsVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """

    def __init__(self):
        GenericComms.__init__(self, BinsVision(self))
        self.defaultDepth = 1.5
        self.sinkingDepth = 2.5

        self.dynServer = DynServer(Config, self.reconfigure)

        if not self.isAlone:
            # Initialize mission planner communication server and client
            self.comServer = rospy.Service("/bins/mission_to_vision",
                                           mission_to_vision,
                                           self.handleSrv)
            rospy.loginfo("Waiting for vision to mission service")
            self.toMission = rospy.ServiceProxy("/bins/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service(timeout=60)

    def handleSrv(self, req):
        if req.start_request:
            rospy.loginfo("Received Start Request")
            self.isAborted = False
            #self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        elif req.abort_request:
            rospy.loginfo("Received Abort Request!!!")
            self.sendMovement(f=0.0, sm=0.0)
            self.isAborted = True
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))

    def drop(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 8)

    def reconfigure(self, config, level):
        self.params = {'hsvLoThresh1' : (config.loH1, config.loS1, config.loV1),
                       'hsvHiThresh1' : (config.hiH1, config.hiS1, config.hiV1),
                       'hsvLoThresh2' : (config.loH2, config.loS2, config.loV2),
                       'hsvHiThresh2' : (config.hiH2, config.hiS2, config.hiV2),
                       'minContourArea' : config.alienMinArea,
                       'adaptiveCoeff' : config.adaptiveCoeff,
                       'adaptiveOffset' : config.adaptiveOffset,
                       'areaThresh' : config.binMinArea }
        self.visionFilter.updateParams()
        return config

def main():
    testCom = Comms()
