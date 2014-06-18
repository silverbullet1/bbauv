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

    def initUI(self):
        main_layout = QHBoxLayout()

        button = QPushButton()
        main_layout.addWidget(button)

        self.initTimer(self.rate)        
        self.setLayout(main_layout)
        self.show()

    def initSub(self):
        self.cam_sub = rospy.Subscriber(rospy.get_param('~image',
            "/front_camera/camera/image_raw"), Image, self.cam_callback)
        self.filter_sub = rospy.Subscriber(rospy.get_param('~filter',
            "/Vision/image_filter"), Image, self.filter_callback)

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

    def initTimer(self,time):
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.updateParams)
        self.timer.start(1000.0 / time)

    def updateParams(self):
        pass
        
    def signal_handler(self, signal, frame):
        sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('Vision', anonymous=True)
    app = QApplication(sys.argv)
    form = Vision()
    signal.signal(signal.SIGINT, form.signal_handler)
    form.show()
    app.exec_()
