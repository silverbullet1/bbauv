#!/usr/bin/env python2

# Tool to help with viewing different types of filters

import roslib; roslib.load_manifest('controlpanel')
import rospy
from sensor_msgs.msg import Image

import Tkinter as tk
import PIL.Image as PImage
import ImageTk

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Helper function to get a readable timestamp
def get_timestamp():
	import time
	return time.strftime('%Y-%m-%d-%H%M%S')


# Helper function to draw a histogram image
def get_hist_img(cv_img):
	hist_bins = 64
	hist_ranges = [(0,255)]

	#hist = cv2.calcHist([cv_img], [0], np.zeros([0,0]), [256], [[0,255]])
	hist, _ = np.histogram(cv_img, hist_bins, (0, 255))
	maxVal = np.max(hist)

	histImg = np.array( [255] * (hist_bins * hist_bins), dtype=np.uint8 ).reshape([hist_bins, hist_bins])
	hpt = int(0.9 * hist_bins)

	for h in range(hist_bins):
		binVal = float(hist[h])
		intensity = int(binVal * hpt / maxVal)
		cv2.line(histImg, (h, hist_bins), (h, hist_bins-intensity), 0)

	return histImg


# Helper class for the various filters
class FilterFrame:
	thumbnail_size = (200, 200)

	def __init__(self, filter_name, tkparent, column, row):
		self.img_filter = None
		self.filter_name = filter_name

		self.tkout = tk.Label(tkparent, text=filter_name)
		self.tkout.grid(column=column, row=row)

	def got_frame(self, cv_img):
		if self.img_filter:
			tmp_img = self.img_filter(cv_img)
		else:
			tmp_img = cv_img

		filtered_pil = PImage.fromarray(tmp_img)
		filtered_pil.thumbnail(self.thumbnail_size, PImage.ANTIALIAS)
		tkfiltered = ImageTk.PhotoImage(filtered_pil)

		self.tkout.config({'image': tkfiltered})
		self.tkout.photo = tkfiltered # prevent garbage collection


class MultiViewer:
	# Convert a ROS Image to the Numpy matrix used by cv2 functions
	def rosimg2cv(self, ros_image):
		# Convert from ROS Image to old OpenCV image
		frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
		# Convert from old OpenCV image to Numpy matrix
		return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype



	def __init__(self, tkroot):
		tkframe = tk.Frame(tkroot)
		tkframe.pack()

		tkvidframe = tk.Frame(tkframe)
		tkvidframe.pack(side=tk.TOP)

		tkbtnframe = tk.Frame(tkframe)
		tkbtnframe.pack(side=tk.BOTTOM)

		self.rec_btn = tk.Button(tkbtnframe, text="REC", command=self.got_rec)
		self.rec_btn.pack(side=tk.LEFT)

		self.vid_writer = None

		self.snap_btn = tk.Button(tkbtnframe, text="Snap", command=self.got_snap)
		self.snap_btn.pack(side=tk.RIGHT)

		self.img_label = tk.Label(tkvidframe, text="Webcam feed")
		self.img_label.pack()

		self.cvbridge = CvBridge()
		self.cur_ros_img = None

		# Set up frames for filtered videos
		tkfilterframe = tk.Frame(tkroot)
		tkfilterframe.pack()

		self.filters = filters = { }

		filters['gray'] = FilterFrame('gray', tkfilterframe, column=0, row=0)
		filters['gray'].img_filter = lambda (cv_img): cv2.cvtColor(cv_img, cv2.cv.CV_BGR2GRAY)

		filters['hsv'] = FilterFrame('hsv', tkfilterframe, column=1, row=0)
		filters['hsv'].img_filter = lambda (cv_img): cv2.cvtColor(cv_img, cv2.cv.CV_BGR2HSV)

		filters['hist'] = FilterFrame('hist', tkfilterframe, column=2, row=0)
		filters['hist'].img_filter = get_hist_img


	# Callback for subscribing to Image topic
	def got_frame(self, ros_image):
		self.cur_ros_img = ros_image

		tmpimg = PImage.frombuffer('RGB', (ros_image.width, ros_image.height), ros_image.data, 'raw', 'BGR')
		tmpimg.thumbnail((300, 300), PImage.ANTIALIAS)

		tkimage = ImageTk.PhotoImage(tmpimg)
		self.img_label.config({'image': tkimage})
		self.img_label.photo = tkimage # prevent garbage collection

		cvimg = self.rosimg2cv(ros_image)
		# Resize the image for display (or maybe should resize each output image)
		if self.vid_writer:
			self.vid_writer.write(cvimg)

		for tkfilter in self.filters.values():
			tkfilter.got_frame(cvimg)


	# Callback when Snap button is pressed
	def got_snap(self):
		if self.cur_ros_img:
			cvimg = self.rosimg2cv(self.cur_ros_img)
			filename = 'snapshot-' + get_timestamp() + '.jpg'
			cv2.imwrite(filename, cvimg)


	# Callback when Rec button is pressed
	def got_rec(self):
		if self.vid_writer:
			# Stop recording
			self.vid_writer = None
			self.rec_btn.config({'text': 'Rec'})
		else:
			# Start recording
			self.rec_btn.config({'text': 'Stop'})
			filename = 'recording-' + get_timestamp() + '.mpg'
			self.vid_writer = cv2.VideoWriter(filename, cv2.cv.CV_FOURCC(*'PIM1'), 24, (320, 240)) #TODO: use actual video framerate, width, height


if __name__ == '__main__':
	tkroot = tk.Tk()
	app = MultiViewer(tkroot)

	rospy.init_node('multi_viewer', anonymous=True)
	rospy.Subscriber("/camera/rgb/image_color", Image, app.got_frame)

	tkroot.mainloop()
