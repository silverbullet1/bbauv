import rospy

from vision import LaneMarkerVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """
    LEFT = 0
    RIGHT = 1

    def __init__(self):
        GenericComms.__init__(self, LaneMarkerVision(comms=self))
        self.chosenLane = self.RIGHT
        self.expectedLanes = 2
        self.defaultDepth = 0.6

        #TODO: Communicate with mission planner

    def handleSrv(self, data):
        pass

def main():
    testCom = Comms()
