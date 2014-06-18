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

from qrangeslider import QRangeSlider 
from histogram import QHistogram
from thresholding import Thresholding 

class Vision(QWidget):
    rate = 20
    hist = None
    thresholder = None
    params = {'hueLow': 0, 'hueHigh': 180,
            'satLow':0, 'satHigh': 255,
            'valLow':0, 'valHigh': 255}

    def __init__(self, parent=None):
        super(Vision, self).__init__(parent)
        self.thresholder = Thresholding()
        self.bridge = CvBridge()
        self.initUI()
        self.initSub()

    def initUI(self):
        main_layout = QHBoxLayout()

        # Colour Map
        colorMap = QLabel()
        pixmap = QPixmap("huescale.png")
        colorMap.setPixmap((pixmap).scaled(710, pixmap.height()))

        # QRange sliders 
        slider_layout = QVBoxLayout()
        hue_layout, self.hue_slider = self.make_slider_box(" Hue", 0, 180)
        sat_layout, self.sat_slider = self.make_slider_box("Sat", 0, 255)
        val_layout, self.val_slider = self.make_slider_box("Val", 0, 255)
        slider_layout.addLayout(hue_layout)
        slider_layout.addLayout(sat_layout)
        slider_layout.addLayout(val_layout)

        # Parameter layout
        lo_h, self.lo_h_box, layout_lo_h = self.make_data_box("loH: ")
        hi_h, self.hi_h_box, layout_hi_h = self.make_data_box("hiH: ")
        lo_s, self.lo_s_box, layout_lo_s = self.make_data_box("loS: ")
        hi_s, self.hi_s_box, layout_hi_s = self.make_data_box("hiS: ")
        lo_v, self.lo_v_box, layout_lo_v = self.make_data_box("loV: ")
        hi_v,  self.hi_v_box, layout_hi_v = self.make_data_box("hiV: ")

        self.changeParamsBtn = QPushButton("Change")
        self.changeParamsBtn.clicked.connect(self.changeParamsBtnHandler)

        params_layout = QHBoxLayout()
        params_layout.addLayout(layout_lo_h)
        params_layout.addLayout(layout_hi_h)
        params_layout.addLayout(layout_lo_s)
        params_layout.addLayout(layout_hi_s)
        params_layout.addLayout(layout_lo_v)
        params_layout.addLayout(layout_hi_v)     
        params_layout.addWidget(self.changeParamsBtn)

        # Vision image layout
        image_layout = QHBoxLayout()   
        video_l = QLabel("<b>Camera</b>")
        filter_l = QLabel("<b>Filter</b>")
        threshold_l = QLabel("<b>Threshold</b>")
        self.video_cb = QLabel()
        self.filter_cb = QLabel()
        self.thres_cb = QLabel()

        video_layout = QVBoxLayout()
        video_layout.addWidget(video_l)
        video_layout.addWidget(self.video_cb)
        filter_layout = QVBoxLayout()
        filter_layout.addWidget(filter_l)
        filter_layout.addWidget(self.filter_cb)
        thres_layout = QVBoxLayout()
        thres_layout.addWidget(threshold_l)
        thres_layout.addWidget(self.thres_cb)

        image_layout.addLayout(video_layout)
        image_layout.addLayout(filter_layout)
        image_layout.addLayout(thres_layout)

        # Histogram layout
        hist_layout = QVBoxLayout()
        hist_l = QLabel("<b>Histogram</b>")
        self.hist = QHistogram()
        self.hist.setParams(self.params)
        hist_layout.addWidget(hist_l)
        hist_layout.addWidget(self.hist)

        # Main frame layout
        frame_layout = QVBoxLayout()
        frame_layout.addWidget(colorMap)
        frame_layout.addLayout(slider_layout)
        frame_layout.addLayout(params_layout)
        frame_layout.addLayout(image_layout)
        
        main_layout.addLayout(frame_layout)
        main_layout.addLayout(hist_layout)

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
            image = self.rosimg2cv(image)
            cvImg = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.updateCameraImage(cvImg)
        except CvBridgeError, e:
            pass

    def filter_callback(self, image):
        try:
            self.q_filter = image 
        except CvBridgeError, e:
            pass

    def updateCameraImage(self, image):
        qimg = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)

        self.hist.updateHist(image)
        self.video_cb.setPixmap(qpm.scaledToHeight(250))

    def rosimg2cv(self, ros_img):
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
        minh, maxh = self.hue_slider.getRange()
        self.params['hueLow'] = minh,
        self.params['hueHigh'] = maxh

        minS, maxS = self.sat_slider.getRange()
        self.params['satLow'] = minS
        self.params['satHigh'] = maxS

        minV, maxV = self.val_slider.getRange()
        self.params['valLow'] = minV
        self.params['valHigh'] = maxV

        self.hist.setParams(self.params)
        self.thresholder.setParams(self.params)
        
    def make_data_box(self, name):
        label = QLabel(name)
        qle = QLineEdit()
        layout = QHBoxLayout()
        layout.addWidget(label)
        layout.addWidget(qle)

        return (label, qle, layout)

    def make_slider_box(self, name, min, max):
        label = QLabel(name)
        qse = QRangeSlider()
        qse.setMax(max)
        qse.setMin(min)
        layout = QHBoxLayout()
        layout.addWidget(qse)
        layout.addWidget(label)

        qse.setStart(min)
        qse.setEnd(max)

        return (layout, qse)

    def changeParamsBtnHandler(self):
        if self.lo_h_box.text() == "":
            lo_h = self.params['hueLow']
        else:
            lo_h = int(self.lo_h_box.text())

        if self.hi_h_box.text() == "":
            hi_h = self.params['hueHigh']
        else:
            hi_h = int(self.hi_h_box.text())

        if self.lo_s_box.text() == "":
            lo_s = self.params['satLow']
        else:
            lo_s = int(self.lo_s_box.text())
        
        if self.hi_s_box.text() == "":
            hi_s = self.params['satHigh']
        else:
            hi_s = int(self.hi_s_box.text())
        
        if self.lo_v_box.text() == "":
            lo_v = self.params['valLow']
        else:
            lo_v = int(self.lo_v_box.text())
        
        if self.hi_v_box.text() == "":
            hi_v = self.params['valHigh']
        else:
            hi_v = int(self.hi_v_box.text())

        self.updateParamsMap(lo_h, hi_h, lo_s, hi_s, lo_v, hi_v)

        # Update sliders
        self.updateSlider()

    def updateSlider(self):
        self.hue_slider.setStart(self.params['hueLow'])
        self.sat_slider.setStart(self.params['satLow'])
        self.val_slider.setStart(self.params['valLow'])
        self.hue_slider.setEnd(self.params['hueHigh'])
        self.sat_slider.setEnd(self.params['satHigh'])
        self.val_slider.setEnd(self.params['valHigh'])

    def updateParamsMap(self, loH, hiH, loS, hiS, loV, hiV):
        self.params['hueLow'] = loH
        self.params['hueHigh'] = hiH
        self.params['satLow'] = loS
        self.params['satHigh'] = hiS
        self.params['valLow'] = loV
        self.params['valHigh'] = hiV

        self.hist.setParams(self.params)
        self.thresholder.setParams(self.params)

    def signal_handler(self, signal, frame):
        sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('Vision', anonymous=True)
    app = QApplication(sys.argv)
    form = Vision()
    signal.signal(signal.SIGINT, form.signal_handler)
    form.show()
    app.exec_()
