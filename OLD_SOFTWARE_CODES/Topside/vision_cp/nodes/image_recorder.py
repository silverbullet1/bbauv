#!/usr/bin/env python2

import roslib; roslib.load_manifest('vision_cp')
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

class ImageRecorder:
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


	# Convert a ROS Image to the Numpy matrix used by cv2 functions
	def rosimg2cv(self, ros_image):
		# Convert from ROS Image to old OpenCV image
		frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
		# Convert from old OpenCV image to Numpy matrix
		return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype


	# Callback for subscribing to Image topic
	def got_frame(self, ros_image):
		self.cur_ros_img = ros_image

		#rospy.loginfo(rospy.get_name() + ": %s" % ros_image.encoding)
		tmpimg = PImage.frombuffer('RGB', (ros_image.width, ros_image.height), ros_image.data, 'raw', 'BGR')
		#tmpimg = tmpimg.transpose(PImage.FLIP_TOP_BOTTOM)

		tkimage = ImageTk.PhotoImage(tmpimg)
		self.img_label.config({'image': tkimage})
		self.img_label.photo = tkimage # prevent garbage collection

		if self.vid_writer:
			cvimg = self.rosimg2cv(ros_image)
			self.vid_writer.write(cvimg)


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
	app = ImageRecorder(tkroot)

	rospy.init_node('image_recorder', anonymous=True)
	rospy.Subscriber("/camera/rgb/image_color", Image, app.got_frame)

	tkroot.mainloop()
