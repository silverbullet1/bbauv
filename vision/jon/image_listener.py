#!/usr/bin/env python2
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from sensor_msgs.msg import Image

import Tkinter as tk
import PIL.Image as PImage
import ImageTk as PImageTk

label = None

def callback(data):
	rospy.loginfo(rospy.get_name() + ": %s" % data.encoding)
	image = PImage.frombuffer('RGB', (data.width, data.height), data.data, 'raw', 'BGR')
	# image = image.transpose(PImage.FLIP_TOP_BOTTOM)
	tkimage = PImageTk.PhotoImage(image)
	label.config({'image': tkimage})
	label.photo = tkimage # prevent garbage collection

def listener():
	global label

	rospy.init_node('image_listener', anonymous=True)
	rospy.Subscriber("/camera/rgb/image_color", Image, callback)

	#rospy.spin()

	root = tk.Tk()
	label = tk.Label(root, text='hello')
	label.pack(side=tk.LEFT)

	root.mainloop()

if __name__ == '__main__':
	listener()

