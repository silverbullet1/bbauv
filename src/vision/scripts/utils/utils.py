import rospy
from cv_bridge import CvBridge, CvBridgeError

class Utils():
    bridge = CvBridge()

    @staticmethod
    def rosimg2cv(ros_img):
        try:
            frame = Utils.bridge.imgmsg_to_cv2(ros_img, ros_img.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)

        return frame

    @staticmethod
    def cv2rosimg(cv_img):
        try:
            return Utils.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    @staticmethod
    def normAngle(angle):
        while angle < 0:
            angle += 360
        return angle % 360

    @staticmethod
    def toHeadingSpace(angle):
        return 90 + angle

    @staticmethod
    def invertAngle(angle):
        if angle < 0: return angle + 180
        else: return angle - 180
