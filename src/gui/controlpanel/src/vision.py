#!/usr/bin/env python

''' Because the control panel is just too clunky '''

import roslib 
import rospy 
import os
import sys
import signal

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt5
from PyQt4.QtWebKit import QGraphicsWebView

from qrangeslider import QRangeSlider 
from histogram import QHistogram
from thresholding import Thresholding 

class Vision(QMainWindow):
	rate = 20
	label_arr = None
    qle_arr = None
    qle_arr2 = None
    layout_arr = None
	hist = None
	thresholder = None
	params = {'hueLow': 0, 'hueHigh': 255,
				'satLow':0, 'satHigh': 255,
				'valLow':0, 'valHigh': 255}

	q_image = None
	q_filter = None

	def __init__(self, parent=None):
		super(Vision, self).__init__(parent)
		self.thresholder = Thresholding()
		self.bridge = CvBridge()
		self.initUI()
		self.initSub()

		self.slider_arr = None

	def initUI(self):
		label_arr = [QLabel(),QLabel(),QLabel(),QLabel(),QLabel(),QLabel(),QLabel()]
        self.slider_arr = [None,None,None,None,None,None]
        self.slider_arr = [None,None,None,None,None,None]
        self.qle_arr = [None,None,None,None,None,None]
        self.qle_arr2 = [None,None,None,None,None,None]
        layout_arr = [None,None,None,None,None,None]
        main_layout = QHBoxLayout()

        ###Channel Params Sliders Declaration
        channel_front_layout = QVBoxLayout()
        channel_front_box = QGroupBox("Threshold Parameters")
        label_arr[0],self.slider_arr[0],self.qle_arr[0],layout_arr[0] = self.make_slider_box(" Hue",0,180)
        label_arr[1],self.slider_arr[1],self.qle_arr[1],layout_arr[1] = self.make_slider_box("Sat",0,255)
        label_arr[2],self.slider_arr[2],self.qle_arr[2],layout_arr[2] = self.make_slider_box("Val",0,255)
     
        self.slider_arr[0].setStart(0)
        self.slider_arr[1].setStart(0)
        self.slider_arr[2].setStart(0)
        self.slider_arr[0].setEnd(180)
        self.slider_arr[1].setEnd(255)
        self.slider_arr[2].setEnd(255)

        self.createColor()
        self.video_layout = QHBoxLayout()
        self.video_filter_l = QLabel("<b>Vision Filter Chain</b>")
        self.video_filter = QLabel()

        lo_h, self.lo_h_box, layout_lo_h = self.make_data_box("loH: ")
        hi_h, self.hi_h_box, layout_hi_h = self.make_data_box("hiH: ")
        lo_s, self.lo_s_box, layout_lo_s = self.make_data_box("loS: ")
        hi_s, self.hi_s_box, layout_hi_s = self.make_data_box("hiS: ")
        lo_v, self.lo_v_box, layout_lo_v = self.make_data_box("loV: ")
        hi_v,  self.hi_v_box, layout_hi_v = self.make_data_box("hiV: ")
        self.changeParamsBtn = QPushButton("Change")
        self.changeParamsBtn.clicked.connect(self.changeParamBtnHandler)
        params_layout = QHBoxLayout()
        params_layout.addLayout(layout_lo_h)
        params_layout.addLayout(layout_hi_h)
        params_layout.addLayout(layout_lo_s)
        params_layout.addLayout(layout_hi_s)
        params_layout.addLayout(layout_lo_v)
        params_layout.addLayout(layout_hi_v)
        params_layout.addWidget(self.changeParamsBtn)    
        
        channel_front_layout.addLayout(self.video_layout)
        channel_front_layout.addWidget(self.video_filter)
        channel_front_layout.addStretch(1)
        channel_front_box.setLayout(channel_front_layout)
        
        channel_layout = QVBoxLayout()
        channel_layout.addWidget(channel_front_box)  

        ### Camera Imagery Declaration
        video_layout = QVBoxLayout()
        self.video_l = QLabel("<b>Camera Selection</b>")
        self.video_cb = QComboBox()
        self.video_cb.addItem("Front Camera")
        self.video_cb.addItem("Bottom Camera")
        self.video_cb.activated[int].connect(self.onActivated)
        
        self.video_top = QLabel()
        self.video_thres = QLabel()
        video_top_l = QLabel("<b>Camera Imagery</b>")
        video_bot_l = QLabel("<b>Thresholded Imagery</b>")
        video_layout.addWidget(self.video_l)
        video_layout.addWidget(self.video_cb)
        video_layout.addWidget(video_top_l)
        video_layout.addWidget(self.video_top)
        video_layout.addWidget(video_bot_l)
        video_layout.addWidget(self.video_thres)
        
        ### Histogram initialization
        hist_layout = QVBoxLayout()
        self.hist = QHistogram()
        hist_l = QLabel("<b>Histogram</b>")
        self.hist.setParams(self.params)
        hist_layout.addWidget(hist_l)
        hist_layout.addWidget(self.hist)
        
        main_layout.addLayout(channel_layout)
        main_layout.addLayout(hist_layout)
        main_layout.addLayout(video_layout)                       

        self.initTimer(self.rate)        
        self.setLayout(main_layout)
        self.show()

	def initSub(self):
		self.cam_sub = rospy.Subscriber(rospy.get_param('~image',
						"/front_camera/camera/image_raw", Image, self.cam_callback))
		self.filter_sub = rospy.Subscriber(rospy.get_param('~filter',
						"/Vision/image_filter", Image, self.filter_callback))

	def cam_callback(self, image):
		try: 
			self.q_image = image
		except CvBridgeError, e:
			pass

	def filter_callback(self, image):
		try:
			self.q_filter = image 
		except CvBridgeError, e:
			pass

	def rosimg2cv(ros_img):
		try:
			frame = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
		except CvBridgeError as e:
			pass

		return frame 

	def signal_handler(self, signal, frame):
		sys.exit(0)

if __name__ == "__main__":
	rospy.init_node('Vision', anonymous=True)
	app = QApplication(sys.argv)
	form = Vision()
	signal.signal(signal.SIGINT, form.signal_handler)
	form.show()
	app.exec_()
