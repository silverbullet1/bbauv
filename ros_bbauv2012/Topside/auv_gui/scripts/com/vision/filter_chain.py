'''
Created on Jul 18, 2013

@author: gohew
'''
import numpy as np
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt
from PyQt4 import QtCore, QtGui, QtSvg
from PyQt4.QtWebKit import QGraphicsWebView
import sys
import os
import cv2
from qrangeslider import QRangeSlider
from com.histogram.histogram import QHistogram
#from com.ui import QRangeSlider

class Vision_filter(QWidget):
    '''
    classdocs
    '''
    rate = 20
    label_arr = None
    qle_arr = None
    qle_arr2 = None
    layout_arr = None
    hist = None
    isFront = 0
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 0, 'hueHigh':255,'valLow':0,'valHigh':255}
   
    #slider_arr = [QSlider]
    def __init__(self):
        '''
        Constructor
        '''
        super(Vision_filter, self).__init__()
        self.initUI()
        
    def initUI(self):
        label_arr = [QLabel(),QLabel(),QLabel(),QLabel(),QLabel(),QLabel(),QLabel()]
        #slider_arr = [QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider(),QRangeSlider()] 
        self.slider_arr = [None,None,None,None,None,None]
        self.qle_arr = [None,None,None,None,None,None]
        self.qle_arr2 = [None,None,None,None,None,None]
        layout_arr = [None,None,None,None,None,None]
        main_layout = QHBoxLayout()
        
        ###Channel Params Sliders Declaration
        channel_front_layout = QVBoxLayout()
        channel_front_box = QGroupBox("Threshold Parameters")
        label_arr[0],self.slider_arr[0],self.qle_arr[0],layout_arr[0] = self.make_slider_box("Hue:",0,255)
        label_arr[1],self.slider_arr[1],self.qle_arr[1],layout_arr[1] = self.make_slider_box("Sat:",0,255)
        label_arr[2],self.slider_arr[2],self.qle_arr[2],layout_arr[2] = self.make_slider_box("Val:",0,255)
        
        self.slider_arr[0].setStart(0)
        self.slider_arr[1].setStart(0)
        self.slider_arr[2].setStart(0)
        self.slider_arr[0].setEnd(255)
        self.slider_arr[1].setEnd(255)
        self.slider_arr[2].setEnd(255)
        #slider_arr[0].minValueChanged.connect(self.sliderChange_1)
        self.createColor()
        channel_front_layout.addWidget(self.view)
        channel_front_layout.addLayout(layout_arr[0])
        channel_front_layout.addLayout(layout_arr[1])
        channel_front_layout.addLayout(layout_arr[2])
        channel_front_layout.addStretch(2)
        channel_front_box.setLayout(channel_front_layout)
        
        '''
        channel_bot_layout = QVBoxLayout()
        channel_bot_box = QGroupBox("Bottom Thresholds")
        label_arr[3],slider_arr[3],qle_arr[3],layout_arr[3] = self.make_slider_box("Hue",0,255)
        label_arr[4],slider_arr[4],qle_arr[4],layout_arr[4] = self.make_slider_box("Sat",0,255)
        label_arr[5],slider_arr[5],qle_arr[5],layout_arr[5] = self.make_slider_box("Val",0,255)
        
        channel_bot_layout.addLayout(layout_arr[3])
        channel_bot_layout.addLayout(layout_arr[4])
        channel_bot_layout.addLayout(layout_arr[5])
        channel_bot_box.setLayout(channel_bot_layout)
        '''
        channel_layout = QVBoxLayout()
        channel_layout.addWidget(channel_front_box)
        #channel_layout.addWidget(channel_bot_box)
        
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
    
    def createColor(self):
        scene = QtGui.QGraphicsScene()
        self.view = QtGui.QGraphicsView(scene)
    
        br = QtSvg.QGraphicsSvgItem(os.getcwd() + '/scripts/HueScale.svg').boundingRect()
    
        webview = QGraphicsWebView()
        webview.load(QtCore.QUrl(os.getcwd() + '/scripts/HueScale.svg'))
        webview.setFlags(QtGui.QGraphicsItem.ItemClipsToShape)
        webview.setCacheMode(QtGui.QGraphicsItem.NoCache)
        webview.resize(br.width()/4, br.height()/4)
    
        scene.addItem(webview)
        self.view.resize(br.width(), br.height())
        self.view.show()
    def initTimer(self,time):
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.updateParams)
        self.timer.start(1000.0 / time)
    
    def updateParams(self):
        min ,max = self.slider_arr[0].getRange()
        self.qle_arr[0].setText("min: " + str(min) + " max: " + str(max))
        
        self.params['hueLow'] = min
        self.params['hueHigh'] = max
        min ,max = self.slider_arr[1].getRange()
        self.params['satLow'] = min
        self.params['satHigh'] = max
        self.qle_arr[1].setText("min: " + str(min) + " max: " + str(max))
        
        min ,max = self.slider_arr[2].getRange()
        self.params['valLow'] = min
        self.params['valHigh'] = max
        self.hist.setParams(self.params)
        self.qle_arr[2].setText("min: " + str(min) + " max: " + str(max))
        
    def threshold_image(self,image):
        hsv_image = cv2.cvtColor(image, cv2.cv.CV_BGR2HSV)
        COLOR_MIN = np.array([self.params['hueLow'],self.params['satLow'],self.params['valLow']],np.uint8)
        COLOR_MAX = np.array([self.params['hueHigh'],self.params['satHigh'],self.params['valHigh']],np.uint8)
        threshold_image = cv2.inRange(hsv_image, COLOR_MIN, COLOR_MAX)
        return threshold_image

    def onActivated(self,index):
        self.isFront = index
        
    def make_slider_box(self, name,min,max):
        label = QLabel(name)
        qse = QRangeSlider()
        qse.setMax(max)
        qse.setMin(min)
        qle = QLabel("")
        #qle2 = QLineEdit()
        layout = QHBoxLayout()
        #qle.setEnabled(False)
        layout.addWidget(label)
        layout.addWidget(qse)
        layout.addWidget(qle)
        #layout.addWidget(qle2)
        #layout.addStretch(1)
        
        return (label,qse, qle,layout)
    
    
        