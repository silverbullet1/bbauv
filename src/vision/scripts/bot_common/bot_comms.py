import rospy
from sensor_msgs.msg import Image
import actionlib

from bbauv_msgs.msg import compass_data, ControllerAction, ControllerGoal
from bbauv_msgs.srv import set_controller

from utils.utils import Utils
import utils.config as config

import signal

class GenericComms:
    """ Class to facilitate communication b/w ROS and task submodules """

    def __init__(self, visionFilter):
        signal.signal(signal.SIGINT, self.userQuit)

        # Initialize default values
        self.inputHeading = 0
        self.curHeading = 0
        self.retVal = None
        self.defaultDepth = 0.0

        # Initialize flags
        self.isAborted = True
        self.isKilled = False
        self.canPublish = False

        #Initialize vision filter
        self.visionFilter = visionFilter

        # Get private params
        self.isAlone = rospy.get_param('~alone', True)
        self.imageTopic = rospy.get_param('~image', config.botCamTopic)

        # Communicate with motion control server
        self.motionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                         ControllerAction)
        try:
            rospy.loginfo("Waiting for LocomotionServer...")
            self.motionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.loginfo("LocomotionServer timeout!")
            self.isKilled = True

        setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        setServer(forward=True, sidemove=True, heading=True, depth=True,
                  pitch=True, roll=True, topside=False, navigation=False)

        # Run straight away if in alone mode
        if self.isAlone:
            self.isAborted = False
            self.canPublish = True

    def register(self):
        self.camSub = rospy.Subscriber(self.imageTopic, Image, self.camCallback)
        self.compassSub = rospy.Subscriber(config.compassTopic,
                                           compass_data,
                                           self.compassCallback)
        self.outPub = rospy.Publisher(config.visionFilterTopic, Image)

    def unregister(self):
        self.camSub.unregister()
        self.compassSub.unregister()

    def camCallback(self, rosImg):
        self.retVal, outImg = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
        if self.canPublish:
            self.outPub.publish(Utils.cv2rosimg(outImg))

    def compassCallback(self, data):
        self.curHeading = data.yaw

    def userQuit(self, signal, frame):
        self.isAborted = True
        self.isKilled = True
        rospy.signal_shutdown("Task manually killed")
    
    def sendMovement(self, f=0.0, sm=0.0, h=None, d=None,
                     timeout=0.4, blocking=False):
        d = d if d else self.defaultDepth
        h = h if h else self.curHeading
        goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h,
                              sidemove_setpoint=sm, depth_setpoint=d)
        self.motionClient.send_goal(goal)
        rospy.loginfo("Moving f:%lf, sm:%lf, h:%lf, d:%lf", f, sm, h, d)

        if blocking:
            self.motionClient.wait_for_result()
        else:
            self.motionClient.wait_for_result(timeout=rospy.Duration(timeout))
