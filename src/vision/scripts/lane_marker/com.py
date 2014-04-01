import roslib; roslib.load_manifest('vision')
import rospy
from sensor_msgs.msg import Image

from bbauv_msgs.msg import compass_data

from utils.utils import Utils
import utils.config as config
from vision import LaneMarkerVision

class Com:
    inputHeading = 0
    curHeading = 0

    def __init__(self):
        # Initialize flags
        self.canPublish = False

        #Initialize vision filter
        self.visionFilter = LaneMarkerVision(self)

        # Get private params
        self.isAlone = rospy.get_param('~alone', True)
        self.imageTopic = rospy.get_param('~image', config.botCamTopic)
    
    def register(self):
        self.camSub = rospy.Subscriber(self.imageTopic, Image, self.camCallback)
        self.compassSub = rospy.Subscriber(config.compassTopic, compass_data, self.compassCallback)
        self.outPub = rospy.Publisher(config.visionTopic, Image)

    def unregister(self):
        self.camSub.unregister()
        self.compassSub.unregister()

    def camCallback(self, rosImg):
        outImg = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
        if self.canPublish:
            self.outPub.publish(Utils.cv2rosimg(outImg))

    def compassCallback(self, data):
        self.curHeading = data.yaw
