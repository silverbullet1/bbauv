import rospy

from vision import BinsVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """

    def __init__(self):
        GenericComms.__init__(self, BinsVision(self))

        #TODO: Add default depth
        #TODO: Communicate with mission planner

    def handleSrv(self, data):
        pass

def main():
    testCom = Comms()
