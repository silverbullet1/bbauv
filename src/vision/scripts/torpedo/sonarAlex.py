#!/usr/bin/env python 

import roslib
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

class Sonar():
	threshold = 130
	bridge = CvBridge()

	def __init__(self):
		self.registerSonar()

	def gotSonarFrame(self, img):
		# width, height = img.shape[:2]
		# Sonar width: 281, height: 219
		rospy.loginfo("Width {}, Height {}".format(width, height))

		return img 

	def registerSonar(self):
		rospy.loginfo("Registering Sonar")
		self.sonarSub = rospy.Subscriber("/sonar_image", Image, self.sonarImageCallback)
		self.sonarPub = rospy.Publisher("/sonar_pub", Image)
		rospy.sleep(rospy.Duration(0.05))

	def sonarImageCallback(self, rosImg):
		outImg = self.gotSonarFrame(self.rosImgToCVMono(rosImg))
		if outImg is not None:
			try:
				self.sonarPub.publish(self.cv2rosimg(outImg))
			except Exception, e:
				pass

	def rosImgToCVMono(self, img):
		try:
			frame = self.bridge.imgmsg_to_cv2(img, desired_encoding="mono8")
		except CvBridgeError as e:
			rospy.logerr(e)
		return frame

	def cv2rosimg(self, img):
		try:
			return self.bridge.cv2_to_imgmsg(img, encoding="mono8")
		except CvBridgeError as e:
			rospy.logerr(e)

def main():
	rospy.init_node('SonarClear')
	sonar = Sonar()
	rospy.spin()
