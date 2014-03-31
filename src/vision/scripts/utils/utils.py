import rospy
from cv_bridge import CvBridge, CvBridgeError

class Utils():
    bridge = CvBridge()

    @staticmethod
    def rosimg2cv(ros_image):
        try:
            frame = Utils.bridge.imgmsg_to_cv2(ros_image, ros_image.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)

        return frame 

    @staticmethod
    def normAngle(angle):
        while angle < 0:
            angle += 360
        return angle % 360

    @staticmethod
    def toHeadingSpace(angle):
        return 90 + angle
