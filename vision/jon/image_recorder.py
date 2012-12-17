#!/usr/bin/env python2
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from sensor_msgs.msg import Image

import Tkinter as tk
import PIL.Image as PImage
import ImageTk as PImageTk

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

recbtn = None
recording = None
label = None
imgdata = None
bridge = CvBridge()

def getCVImg(ros_image):
	# Convert from ROS Image to old OpenCV image
	frame = bridge.imgmsg_to_cv(ros_image, ros_image.encoding)

	# Convert from old OpenCV image to Numpy matrix
	return np.array(frame, dtype=np.uint8)

def callback(data):
	global label, imgdata, recording

	imgdata = data

	#rospy.loginfo(rospy.get_name() + ": %s" % data.encoding)
	image = PImage.frombuffer('RGB', (data.width, data.height), data.data, 'raw', 'BGR')
	#image = image.transpose(PImage.FLIP_TOP_BOTTOM)
	tkimage = PImageTk.PhotoImage(image)
	label.config({'image': tkimage})
	label.photo = tkimage # prevent garbage collection

	if recording:
		cvimg = getCVImg(imgdata)
		recording.write(cvimg)

def click():
	global imgdata

	if imgdata != None:
		image = getCVImg(imgdata)
		cv2.imwrite('sample.jpg', image)

def rec():
	global recording, recbtn

	if recording:
		recording = None
		recbtn.config({'text': 'Rec'})
		# Stop recording
	else:
		# Start recording
		recbtn.config({'text': 'Stop'})
		recording = cv2.VideoWriter('sample.mpg', cv2.cv.CV_FOURCC('P', 'I', 'M', '1'), 24, (320, 240))


def listener():
	global label, recbtn

	rospy.init_node('image_listener', anonymous=True)
	rospy.Subscriber("/camera/rgb/image_color", Image, callback)

	root = tk.Tk()
	label = tk.Label(root, text='hello')
	label.pack(side=tk.LEFT)

	snapbtn = tk.Button(root, text='Snap', command=click)
	snapbtn.pack(side=tk.BOTTOM)

	recbtn = tk.Button(root, text='Rec', command=rec)
	recbtn.pack(side=tk.BOTTOM)

	root.mainloop()

if __name__ == '__main__':
	listener()

