#!/usr/bin/env python2

# Code to follow a black line

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image

# from bbauv_msgs.msg import compass_data
# from bbauv_msgs.msg import controller_input

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Publisher to controller input
movementPub = None
def publishMovement(movement):
	movementPub.publish(movement)


# Helper function to draw a histogram image
def get_hist_img(cv_img):
	hist_bins = 256
	hist_ranges = [(0,255)]

	hist, _ = np.histogram(cv_img, hist_bins, (0, 255))
	maxVal = np.max(hist)

	histImg = np.array( [255] * (hist_bins * hist_bins), dtype=np.uint8 ).reshape([hist_bins, hist_bins])
	hpt = int(0.9 * hist_bins)

	for h in range(hist_bins):
		binVal = float(hist[h])
		intensity = int(binVal * hpt / maxVal)
		cv2.line(histImg, (h, hist_bins), (h, hist_bins-intensity), 0)

	return histImg


# Every class for the various states should have the following format:
#	class FollowerState:
#		# Callback when an image frame is received
#		# cvimg is the image, rectData is a dictionary containing the bounding rect data
#		def gotFrame(self, cvimg, rectData):
#			...
#			returns either this state (self) or a new state

class LookForLineState:
	def __init__(self):
		rospy.loginfo("Looking for line")

	def gotFrame(self, cvimg, rectData):
		if rectData['maxRect']:
			return StraightLineState()
		#TODO: Keep spinning right
		print("Spin right!")
		return self

class TemporaryReverseState:
	def __init__(self, secondsToReverse, nextState):
		rospy.loginfo("Reversing for " + str(secondsToReverse) + " secs")
		self.transitionTime = rospy.get_time() + secondsToReverse
		self.nextState = nextState

	def gotFrame(self, cvimg, rectData):
		if rospy.get_time() > self.transitionTime:
			return self.nextState()
		#TODO: Keep reversing
		print("Reverse!")
		return self

class StraightLineState:
	def __init__(self):
		rospy.loginfo("Following a straight line")

	def gotFrame(self, cvimg, rectData):
		if rectData['maxRect'] == None:
			# Lost the line, so reverse a bit, then look again
			return TemporaryReverseState(1.5, LookForLineState)
		#TODO: Keep going forward
		print("Go forward!")
		return self


# Actual line-following class
class LineFollower:
	# Convert a ROS Image to the Numpy matrix used by cv2 functions
	def rosimg2cv(self, ros_image):
		# Convert from ROS Image to old OpenCV image
		frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
		# Convert from old OpenCV image to Numpy matrix
		return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype


	def __init__(self):
		self.cvbridge = CvBridge()

		# Initial following state
		self.heading = 0.0
		self.state = LookForLineState()

		# Configurable parameters
		self.params = { 'grayLow': 0, 'grayHigh': 40, 'contourMinArea': 300 }

		# Set up param configuration window
		def paramSetter(key):
			def setter(val):
				self.params[key] = val
			return setter
		cv2.namedWindow("settings", cv2.CV_WINDOW_AUTOSIZE)
		cv2.createTrackbar("Gray Low:", "settings", self.params['grayLow'], 255, paramSetter('grayLow'));
		cv2.createTrackbar("Gray High:", "settings", self.params['grayHigh'], 255, paramSetter('grayHigh'));
		cv2.createTrackbar("Min contour area:", "settings", self.params['contourMinArea'], 3000, paramSetter('contourMinArea'));

		## Example filters
		## Conversion to HSV
		#hsvfilter = lambda (cv_img): cv2.cvtColor(cv_img, cv2.cv.CV_BGR2HSV)

		## Grayscale image that has its histogram equalized
		#equalhistfilter = lambda (_): cv2.equalizeHist(grayfilter.image)

		## Original image - its Laplacian
		#tmpkernel = np.array([[0., -1., 0.],[-1., 5., -1.], [0., -1., 0.]])
		#sublaplacefilter = lambda (cv_img): cv2.filter2D(cv_img, -1, tmpkernel)

		## Canny (edge detection)
		#cannyfilter = lambda (_): cv2.threshold(cv2.Canny(grayfilter.image, 125, 350), 128, 255, cv2.THRESH_BINARY_INV)[1]

		## R,G,B
		#rfilter = lambda (cv_img): cv2.split(cv_img)[2]
		#gfilter = lambda (cv_img): cv2.split(cv_img)[1]
		#bfilter = lambda (cv_img): cv2.split(cv_img)[0]


	# Callback for subscribing to Image topic
	def gotRosFrame(self, rosImage):
		cvimg = self.rosimg2cv(rosImage)
		self.gotFrame(cvimg)

	def gotFrame(self, cvimg):
		# Perform some processing before passing to state

		# Find the black regions
		imgGray = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2GRAY)
		imgBW = cv2.inRange(imgGray, np.array(self.params['grayLow']), np.array(self.params['grayHigh']))

		# Make a copy because findContours modifies the original image
		imgBW2 = imgBW.copy()

		# Retrieve the contours of the black regions
		contours, _ = cv2.findContours(imgBW2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		# Get the largest contour and find its bounding rectangle
		maxArea = 0
		maxRect = None
		for contour in contours:
			curArea = cv2.contourArea(contour)
			if curArea >= self.params['contourMinArea'] and curArea > maxArea:
				maxArea = curArea
				maxRect = cv2.minAreaRect(contour)

		rectData = { 'maxRect': maxRect }

		# maxRect is a tuple: ( (x,y), (w,h), theta )
		# Perform operations on the largest bounding rect
		if maxRect:
			# Obtain the actual corners of the box
			points = cv2.cv.BoxPoints(maxRect)
			# Draw the lines
			for i in range(4):
				# The line function doesn't accept floating point values
				pt1 = (int(points[i][0]), int(points[i][1]))
				pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
				cv2.line(imgBW, pt1, pt2, 255, 1)

			# Prepare other data about the bounding rect for the state machine
			rectData['points'] = points
			rectData['edges'] = edges = [ np.array(points[1]) - np.array(points[0]), np.array(points[2]) - np.array(points[1]) ]
			# Calculate angle from the vertical
			if cv2.norm(edges[0]) > cv2.norm(edges[1]):
				rectData['angle'] = math.atan(edges[0][0] / edges[0][1]) / math.pi * 180
			else:
				rectData['angle'] = math.atan(edges[1][0] / edges[1][1]) / math.pi * 180

		self.state = self.state.gotFrame(imgGray, rectData)

		cv2.imshow("src", cvimg)
		cv2.imshow("gray", imgGray)
		cv2.imshow("bw", imgBW)


	# Callback for subscribing to compass data
	def gotHeading(self, msg):
		self.heading = msg.yaw


if __name__ == '__main__':
	rospy.init_node('line_follower', anonymous=True)
	loopRateHz = rospy.get_param('~loopHz', 20)
	imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_raw')
	compassTopic = rospy.get_param('~compass', '/os5000_data')

	app = LineFollower()

	rospy.Subscriber(imageTopic, Image, app.gotRosFrame)
	# rospy.Subscriber(compassTopic, compass_data, app.gotHeading)
	# movementPub = rospy.Publisher('/line_follower', controller_input)

	r = rospy.Rate(loopRateHz)
	while not rospy.is_shutdown():
		key = cv2.waitKey(20)
		if key == 27: # Exit on getting the Esc key
			break
		r.sleep()
