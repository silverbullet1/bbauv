import rospy
from cv_bridge import CvBridgeError
import config

def rosimg2cv(ros_image):
    try:
        frame = config.bridge.imgmsg_to_cv2(ros_image, ros_image.encoding)
    except CvBridgeError as e:
        rospy.logerr(e)

    return frame 
