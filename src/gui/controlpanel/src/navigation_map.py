'''Draw navigation map'''

import numpy as np
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt
from PyQt4 import QtCore, QtGui, QtSvg

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt

import rospy

import sys
import os
import random

class Navigation_Map(QWidget):
    
    def __init__(self):
        super(Navigation_Map, self).__init__()
        self.data = []
        self.rate = 20
        self.initUI()
    
    def initUI(self):
        map_label = QLabel("<b>Map of the World</b>")
        refresh_button = QPushButton("&Refresh")
        
        map_layout = QHBoxLayout()
        map_layout.addWidget(map_label)
        map_layout.addWidget(refresh_button)
        map_layout.addStretch()
        
        refresh_button.clicked.connect(self.refreshBtnHandler)
        
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.refreshBtnHandler()
                
        main_layout = QVBoxLayout()
        main_layout.addLayout(map_layout)
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.canvas)
        
        self.initTimer(self.rate)
        self.setLayout(main_layout)
        self.show()
    
    def initTimer(self, time):
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.updateParams)
        self.timer.start(1000.0 / time)
    
    def updateParams(self):
        pass
        
    def refreshBtnHandler(self):
        # random data
        #data = [random.random() for i in range(100)]
        for i in range(100):
            self.data.append(random.random())

        # create an axis
        ax = self.figure.add_subplot(111)

        # discards the old graph
        ax.hold(False)

        # plot data
        ax.plot(self.data, '*-')
        # refresh canvas
        self.canvas.draw()
        rospy.loginfo("Refreshed Data")