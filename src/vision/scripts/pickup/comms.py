import rospy

from bbauv_msgs.srv import mission_to_vision, vision_to_mission, \
        mission_to_visionResponse
from bbauv_msgs.msg import controller

from vision import PickupVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """

    def __init__(self):
        GenericComms.__init__(self, PickupVision())

        if not self.isAlone:
            # Initialize mission planner communication server and client
            self.comServer = rospy.Service("/lane/mission_to_vision",
                                           mission_to_vision,
                                           self.handleSrv)
            rospy.loginfo("Waiting for vision to mission service")
            self.toMission = rospy.ServiceProxy("/lane/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service(timeout=60)

    def handleSrv(self, req):
        if req.start_request:
            rospy.loginfo("Received Start Request")
            self.isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint
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

def main():
    pass
    # testCom = Comms()
