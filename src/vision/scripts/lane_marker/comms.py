import roslib; roslib.load_manifest('vision')
import rospy
from dynamic_reconfigure.server import Server as DynServer

from bbauv_msgs.msg import controller
from bbauv_msgs.srv import mission_to_visionResponse
from utils.config import laneConfig as Config

from vision import LaneMarkerVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """
    LEFT = 0
    RIGHT = 1

    def __init__(self, isAcoustic):
        GenericComms.__init__(self, LaneMarkerVision(comms=self))
        self.chosenLane = self.RIGHT
        self.expectedLanes = 1

        self.dynServer = DynServer(Config, self.reconfigure)
        self.isAcoustic = isAcoustic

        if self.isAcoustic:
            self.detectingBox = True
            self.defaultDepth = 0.6
            self.laneSearchDepth = 0.0
        else:
            self.detectingBox = False
            self.defaultDepth = 0.6

        if not self.isAlone:
            if self.isAcoustic:
                self.initComms("laneAcoustic")
            else:
                self.initComms("lane")

    def handleSrv(self, req):
        if req.start_request:
            rospy.loginfo("Received Start Request")
            self.isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            self.expectedLanes = req.numLanes
            self.chosenLane = self.LEFT if req.chosenLane == 0 else self.RIGHT
            rospy.loginfo("Input: d={}, h={}, num={}, chosen={}".format(
                self.defaultDepth, self.inputHeading,
                self.expectedLanes, self.chosenLane))
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        elif req.abort_request:
            rospy.loginfo("Received Abort Request")
            self.sendMovement(f=0.0, sm=0.0)
            self.isAborted = True
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
    def reconfigure(self, config, level):
        rospy.loginfo("Receive dynamic reconfigure request")
        self.params = {'hsvLoThresh1' : (config.loH, config.loS, config.loV),
                       'hsvHiThresh1' : (config.hiH, config.hiS, config.hiV),
                       'minContourArea' : config.minArea,
                       'yellowLoThresh': (config.yellowLoH,
                                          config.yellowLoS,
                                          config.yellowLoV),
                       'yellowHiThresh': (config.yellowHiH,
                                          config.yellowHiS,
                                          config.yellowHiV),
                       'minBoxArea': config.minBoxArea,
                       'ratioBound': config.ratioBound}
        self.visionFilter.updateParams()
        return config

def main():
    pass
