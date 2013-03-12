#!/usr/bin/env python2

# Code to follow a black line

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image

from bbauv_msgs.msg import compass_data
from bbauv_msgs.msg import controller_input

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from collections import deque


# CONSTANTS
DEPTH_POINT = 1.1
secondsToRun = 2.25 * 60
x_strip_threshold = 0.2


# Publisher to controller input
movementPub = None
def publishMovement(movement):
	movementPub.publish(movement)

# Helper function to normalize heading
def normHeading(heading):
	if heading > 360:
		return heading - 360
	if heading < 0:
		return heading + 360
	return heading

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
		# Keep spinning right
		msg = controller_input()
		msg.depth_setpoint = DEPTH_POINT
#		msg.heading_setpoint = rectData['heading'] + 10
		publishMovement(msg)
		return self

class TemporaryState:
	def __init__(self, secondsToReverse, nextState, speed=-0.6):
		rospy.loginfo("Reversing for " + str(secondsToReverse) + " secs")
		self.transitionTime = rospy.get_time() + secondsToReverse
		self.nextState = nextState
		self.speed = speed

	def gotFrame(self, cvimg, rectData):
		if rospy.get_time() > self.transitionTime:
			return self.nextState()
		# Keep reversing
		msg = controller_input()
		msg.depth_setpoint = DEPTH_POINT
		msg.heading_setpoint = rectData['heading']
		msg.forward_setpoint = self.speed
		publishMovement(msg)
		return self


class DiveState:
	def __init__(self, secondsToDive, nextState):
		rospy.loginfo("Diving for " + str(secondsToDive) + " secs")
		self.transitionTime = rospy.get_time() + secondsToDive
		self.nextState = nextState

	def gotFrame(self, cvimg, rectData):
		if rospy.get_time() > self.transitionTime:
			return self.nextState()
		# Dive
		msg = controller_input()
		msg.depth_setpoint = DEPTH_POINT
		msg.heading_setpoint = rectData['heading']
		publishMovement(msg)
		return self

class SurfaceState:
	def __init__(self, heading):
		rospy.loginfo("Surfacing")

		msg = controller_input()
		msg.depth_setpoint = 0.2
		msg.heading_setpoint = heading
		publishMovement(msg)

	def gotFrame(self, cvimg, rectData):
		return self

hist = deque([])

class StraightLineState:
	def __init__(self):
		rospy.loginfo("Following a straight line")

	def gotFrame(self, cvimg, rectData):
		if rectData['maxRect'] == None:
			# Lost the line, so reverse a bit, then look again
			return TemporaryState(0.5, LookForLineState)

		screen_width = cvimg.shape[1]
		screen_center_x = screen_width / 2

		delta_x = float(rectData['maxRect'][0][0] - screen_center_x) / screen_width

		msg = controller_input()
		msg.depth_setpoint = DEPTH_POINT

		# if the rect is too far off centre, do aggressive sidemove
		if abs(delta_x) > 0.3:
			rospy.loginfo('Box too far off centre! Aggressive sidemove')
			msg.heading_setpoint = normHeading(rectData['heading'] - rectData['angle'])
			msg.sidemove_setpoint = math.copysign(1.0, -delta_x)
			publishMovement(msg)
			return self

		print(rectData['angle'])
		# Based on previous angle, determine if the new angle should be pointing the opposite direction
#		if len(hist) >= 1 and abs(rectData['heading'] - rectData['angle']) > 45:
		if len(hist) >= 1:
			oppAngle = rectData['angle'] + (-180 if rectData['angle'] > 0 else 180)
			if abs(rectData['angle'] - hist[0]) > abs(oppAngle - hist[0]):
				rectData['angle'] = oppAngle
				print('new angle: ' + str(oppAngle))

		hist.appendleft(rectData['angle'])

		if delta_x < -x_strip_threshold:
			msg.sidemove_setpoint = 0.5
		elif delta_x > x_strip_threshold:
			msg.sidemove_setpoint = -0.5

		if abs(rectData['angle']) < 10:
			# Keep going forward
			msg.heading_setpoint = rectData['heading']
			msg.forward_setpoint = 0.9
			rospy.loginfo('forward!')
		else:
			# Correct for angle
#			if rectData['angle'] > 45:
#				return TemporaryState(0.3, StraightLineState, speed=-0.4)

			if msg.sidemove_setpoint == 0 and abs(rectData['angle']) > 10:
				msg.sidemove_setpoint = rectData['angle'] / 60 * 0.2

			angle_diff = rectData['angle']
			if abs(angle_diff) > 30:
				angle_diff = math.copysign(30, angle_diff)
			msg.heading_setpoint = normHeading(rectData['heading'] - angle_diff)

		publishMovement(msg)
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
		self.enabled = False

		# Configurable parameters
		self.params = { 'grayLow': 0, 'grayHigh': 10, 'contourMinArea': 300 }

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
		## Original image - its Laplacian
		#tmpkernel = np.array([[0., -1., 0.],[-1., 5., -1.], [0., -1., 0.]])
		#sublaplacefilter = lambda (cv_img): cv2.filter2D(cv_img, -1, tmpkernel)

		## Canny (edge detection)
		#cannyfilter = lambda (_): cv2.threshold(cv2.Canny(grayfilter.image, 125, 350), 128, 255, cv2.THRESH_BINARY_INV)[1]


	# Callback for subscribing to Image topic
	def gotRosFrame(self, rosImage):
		cvimg = self.rosimg2cv(rosImage)
		self.gotFrame(cvimg)

	def gotFrame(self, cvimg):
		# Perform some processing before passing to state

		# Ignore all blue stuff
		channels = cv2.split(cvimg)
		imgNew = cv2.merge([np.zeros(channels[1].shape, channels[1].dtype), channels[1], channels[2]])

		# Find the black regions
		imgGray = cv2.cvtColor(imgNew, cv2.cv.CV_BGR2GRAY)
		imgGray = cv2.equalizeHist(imgGray)

		imgBW = cv2.inRange(imgGray, np.array(self.params['grayLow']), np.array(self.params['grayHigh']))

		structuringElt = cv2.getStructuringElement(cv2.MORPH_RECT,
					(3,3), (1,1))
		imgBW = cv2.dilate(imgBW, structuringElt)
		imgBW = cv2.erode(imgBW, structuringElt)

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

		rectData = { 'maxRect': maxRect, 'heading': self.heading }

		# maxRect is a tuple: ( (center_x,center_y), (w,h), theta )
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

			if rectData['angle'] == 90:
				if maxRect[0][0] > (cvimg.shape[1] / 2):
					rectData['angle'] = -90

		if self.enabled:
			self.state = self.state.gotFrame(imgGray, rectData)

		cv2.imshow("src", imgNew)
		cv2.imshow("gray", imgGray)
		cv2.imshow("bw", imgBW)


	# Callback for subscribing to compass data
	def gotHeading(self, msg):
		self.heading = msg.yaw

	# Start by diving
	def start(self):
		self.state = DiveState(0.2, StraightLineState)
		self.enabled = True

	# End by surfacing
	def stop(self):
		self.state = SurfaceState(self.heading)
		self.enabled = False
		


if __name__ == '__main__':
	rospy.init_node('line_follower', anonymous=True)
	loopRateHz = rospy.get_param('~loopHz', 20)
	imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_raw')
	compassTopic = rospy.get_param('~compass', '/os5000_data')

	app = LineFollower()
	app.start()

	rospy.Subscriber(imageTopic, Image, app.gotRosFrame)
	rospy.Subscriber(compassTopic, compass_data, app.gotHeading)
	movementPub = rospy.Publisher('/line_follower', controller_input)

	endTime = 0

	r = rospy.Rate(loopRateHz)
	while not rospy.is_shutdown():
		key = cv2.waitKey(20)
		if key == 27: # Exit on getting the Esc key
			break

#		# Hit space to start!
#		if key == 32:
#			app.start()
#			endTime = rospy.get_time() + secondsToRun
#
#		if key == ord('q'):
#			app.stop()
#
#		if endTime <= rospy.get_time() and app.enabled:
#			app.stop()
		r.sleep()
