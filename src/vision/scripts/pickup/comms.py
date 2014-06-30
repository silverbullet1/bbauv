import rospy
from dynamic_reconfigure.server import Server as DynServer

from bbauv_msgs.srv import mission_to_vision, vision_to_mission, \
        mission_to_visionResponse
from bbauv_msgs.msg import controller, manipulator, depth
from utils.config import pickupConfig as Config

from vision import PickupVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """
    def __init__(self, taskMode='pickup'):
        GenericComms.__init__(self, PickupVision(self))

        if taskMode == 'pickup':
            self.defaultDepth = 0.6
            self.sinkingDepth = 2.0
            self.grabbingDepth = 2.9
            self.lastDepth = 3.80
            self.grabbingArea = 20000

            self.visionMode = PickupVision.SITE
        elif taskMode == 'drop':
            self.defaultDepth = 1.2
            self.sinkingDepth = 2.5

            self.visionMode = PickupVision.BOX

        self.depthSub = rospy.Subscriber("/depth", depth, self.depthCb)
        self.maniPub = rospy.Publisher("/manipulators", manipulator)
        self.dynServer = DynServer(Config, self.reconfigure)

        if not self.isAlone:
            self.initComms(taskMode)

    def initComms(self, name):
        # Initialize mission planner communication server and client
        self.comServer = rospy.Service("/{}/mission_to_vision".format(name),
                                       mission_to_vision,
                                       self.handleSrv)
        rospy.loginfo("Waiting for vision to mission service")
        self.toMission = rospy.ServiceProxy("/{}/vision_to_mission".format(name),
                                            vision_to_mission)
        self.toMission.wait_for_service()

    def depthCb(self, data):
        self.curDepth = data.depth

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
            rospy.loginfo("Received Abort Request")
            self.sendMovement(f=0.0, sm=0.0)
            self.isAborted = True
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))

    def reconfigure(self, config, level):
        rospy.loginfo("Received reconfigure request")
        self.params = {'yellowLoThresh': (config.yellowLoH,
                                          config.yellowLoS,
                                          config.yellowLoV),
                       'yellowHiThresh': (config.yellowHiH,
                                          config.yellowHiS,
                                          config.yellowHiV),
                       'greenLoThresh': (config.greenLoH,
                                         config.greenLoS,
                                         config.greenLoV),
                       'greenHiThresh': (config.greenHiH,
                                         config.greenHiS,
                                         config.greenHiV),
                       'redLoThresh1': (config.redLoH1,
                                        config.redLoS1,
                                        config.redLoV1),
                       'redHiThresh1': (config.redHiH1,
                                        config.redHiS1,
                                        config.redHiV1),
                       'redLoThresh2': (config.redLoH2,
                                        config.redLoS1,
                                        config.redLoV1),
                       'redHiThresh2': (config.redHiH2,
                                        config.redHiS1,
                                        config.redHiV1),

                       'minSiteArea': config.minSiteArea,
                       'minContourArea' : config.minArea,
                       'maxContourArea' : config.maxArea}
        self.grabbingArea = config.grabbingArea
        self.visionFilter.updateParams()
        return config

    def grab(self):
        self.maniPub.publish(0 | 4)

    def drop(self):
        self.maniPub.publish(0)


def main():
    pass
    # testCom = Comms()
