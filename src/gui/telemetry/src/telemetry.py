#!/usr/bin/env python2
import roslib
import rospy
import os

from bbauv_msgs.srv import *
from bbauv_msgs.msg import *
from nav_msgs.msg import Odometry
import pynotify 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, quaternion_about_axis

from bbauv_msgs.msg import openups_stats
from math import pi
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt
import Queue
import threading

import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image

from bbauv_msgs.msg._thruster import thruster
from std_msgs.msg._Float32 import Float32
from std_msgs.msg._Int8 import Int8
from filter_chain import Vision_filter

class AUV_gui(QMainWindow):
    isLeak = False
    main_frame = None
    compass = None 
    heading_provider = None
    depth_thermo = None
    client = None
    movebase_client = None
    yaw = 0
    depth = 0
    pos_x = 0
    pos_y = 0
    isAlert = [True,True,True,True]
    cvRGBImg_front = None
    cvRGBImg_rfront = None
    cvRGBImg_bot = None
    isArmed = False
    update_freq = 40
    vision_filter_frame = None
    filter_image = None
    q_orientation = Queue.Queue()
    q_depth = Queue.Queue() 
    q_earth_pos = Queue.Queue()
    q_rel_pos = Queue.Queue()
    q_controller = Queue.Queue()
    q_hull_status = Queue.Queue()
    q_mani = Queue.Queue()
    q_mode = Queue.Queue()
    q_controller_feedback = Queue.Queue()
    q_thruster = Queue.Queue()
    q_openups = Queue.Queue()
    q_temp = Queue.Queue()
    q_altitude = Queue.Queue()
    q_image_bot = None
    q_image_front = None
    q_image_rfront = None
    data = {'yaw': 0, 'pitch' : 0,'roll':0, 'depth': 0,'mode':0, 'attitude':0,
            'pressure':0,'forward_setpoint':0,'sidemove_setpoint':0,
            'heading_setpoint':0,'depth_setpoint':0,'altitude':0,'heading_error':0,'openups':openups_stats(),
            'forward_error':0,'sidemove_error':0,'temp':0,'depth_error':0,'goal_id':"None",'thrusters':thruster(),
            'hull_status':hull_status(),'status':-1,'earth_pos':Odometry(),'rel_pos':Odometry(),'manipulators':manipulator()}
    counter = 0
    def __init__(self, parent=None):
        super(AUV_gui, self).__init__(parent)
        
        self.main_tab = QTabWidget()
        self.main_frame = QWidget()
        self.vision_filter_frame = Vision_filter()
        self.main_tab.addTab(self.main_frame, "Telemetry")
        self.main_tab.addTab(self.vision_filter_frame, "Vision Filter")
        
        goalBox =  QGroupBox("Goal Setter")
        depth_l , self.depth_box, layout4 = self.make_data_box("Depth:       ")
        sidemove_l, self.sidemove_box,layout2 = self.make_data_box("Sidemove:")
        forward_l, self.forward_box,layout1 = self.make_data_box("Forward:   ")
        heading_l, self.heading_box,layout3 = self.make_data_box("Heading:   ")
        
        rel_heading_chk, self.rel_heading_chkbox,layout5 =self.make_data_chkbox("Relative:")
        rel_depth_chk, self.rel_depth_chkbox,layout6 =self.make_data_chkbox("Relative:")
        goal_heading_layout = QHBoxLayout()
        goal_heading_layout.addLayout(layout3)
        goal_heading_layout.addLayout(layout5)
        goal_depth_layout = QHBoxLayout()
        goal_depth_layout.addLayout(layout4)
        goal_depth_layout.addLayout(layout6)
        
        goal_layout = QVBoxLayout()
        goal_layout.addLayout(layout1)
        goal_layout.addLayout(layout2)
        goal_layout.addLayout(goal_heading_layout)
        goal_layout.addLayout(goal_depth_layout)
        
        # Buttons Layout
        okButton = QPushButton("&Start Goal")
        cancelButton = QPushButton("&End Goal")
        hoverButton = QPushButton("&Hover")
        surfaceButton = QPushButton("S&urface")
        homeButton = QPushButton("Home &Base")
        self.modeButton = QPushButton("Default")
        mode_l, self.l_mode,mode_layout = self.make_data_box("Loc Mode:")
        self.l_mode.setAlignment(Qt.AlignCenter)
        self.l_mode.setEnabled(False)
        self.armButton = QCommandLinkButton("NOT ARMED")
        fireButton = QPushButton("&Fire")
        self.check1 = QCheckBox("Left Dropper")
        self.check2 = QCheckBox("Right Dropper")
        self.check3 = QCheckBox("Left Torpedo")
        self.check4 = QCheckBox("Right Torpedo")
        self.check5 = QCheckBox("Grabber")
        self.check6 = QCheckBox("Linear")
        self.check7 = QCheckBox("Rotary")
        
        mani_layout = QVBoxLayout()
        mani_layout.addWidget(self.check1)
        mani_layout.addWidget(self.check2)
        mani_layout.addWidget(self.check3)
        mani_layout.addWidget(self.check4)
        mani_layout.addWidget(self.check5)
        mani_layout.addWidget(self.check6)
        mani_layout.addWidget(self.check7)
        mani_layout.addWidget(self.armButton)
        mani_layout.addWidget(fireButton)
        maniBox = QGroupBox("Manipulators Console")
        maniBox.setLayout(mani_layout)
        
        self.armButton.clicked.connect(self.armBtnHandler)
        fireButton.clicked.connect(self.fireBtnHandler)
        okButton.clicked.connect(self.startBtnHandler)
        cancelButton.clicked.connect(self.endBtnHandler) 
        surfaceButton.clicked.connect(self.surfaceBtnHandler)
        hoverButton.clicked.connect(self.hoverBtnHandler)
        homeButton.clicked.connect(self.homeBtnHandler)
        self.modeButton.clicked.connect(self.modeBtnHandler)
        vbox = QVBoxLayout()
        #hbox.addStretch(1)
        vbox.addWidget(okButton)
        vbox.addWidget(cancelButton)
        vbox2 = QVBoxLayout()
        #hbox.addStretch(1)
        vbox2.addWidget(hoverButton)
        vbox2.addWidget(surfaceButton)
        vbox2.addWidget(homeButton)
        
        vbox3 = QVBoxLayout()
        #hbox.addStretch(1)
        vbox3.addLayout(mode_layout)
        vbox3.addWidget(self.modeButton)
        
        goal_gui_layout = QHBoxLayout()
        goal_gui_layout.addLayout(goal_layout)
        
        goalBox_layout = QVBoxLayout()
        goalBox_layout.addLayout(goal_gui_layout)
        
        goalBtn_layout = QHBoxLayout()
        goalBtn_layout.addLayout(vbox)
        goalBtn_layout.addLayout(vbox2)
        goalBtn_layout.addLayout(vbox3)
        #goalBox_layout.addStretch(1)
        goalBox_layout.addLayout(goalBtn_layout)
        
        #vbox.addStretch(1)
        #vbox2.addStretch(1)
        vbox3.addStretch(1)
        goalBox.setLayout(goalBox_layout)
        self.compass = Qwt.QwtCompass()
        self.compass.setGeometry(0,0,200,200)
        self.compass.setLineWidth(4)
        self.compass.setMode(Qwt.QwtCompass.RotateNeedle)
        rose = Qwt.QwtSimpleCompassRose(16, 2)
        rose.setWidth(0.15)
        self.compass.setRose(rose)
        self.compass.setNeedle(Qwt.QwtCompassMagnetNeedle(
                Qwt.QwtCompassMagnetNeedle.ThinStyle))
        self.compass.setValue(220.0)
        self.compass.setScale(36, 5, 0)

        self.heading_provider = Qwt.QwtCompass()
        self.heading_provider.setLineWidth(4)
        self.heading_provider.setMode(Qwt.QwtCompass.RotateNeedle)
        rose = Qwt.QwtSimpleCompassRose(16, 2)
        rose.setWidth(0.15)
        self.heading_provider.setRose(rose)
        self.heading_provider.setNeedle(Qwt.QwtCompassMagnetNeedle(
                Qwt.QwtCompassMagnetNeedle.ThinStyle))
        compass_l = QLabel("Current")
        heading_l = QLabel("User Goal")
        compass_l.setAlignment(Qt.AlignHCenter)
        heading_l.setAlignment(Qt.AlignHCenter)
        compass_layout = QHBoxLayout()
        #compass_layout.addWidget(compass_l)
        current_layout = QVBoxLayout()
        current_layout.addWidget(self.compass)
        current_layout.addWidget(compass_l)
        current_layout.addStretch(1)
        user_layout = QVBoxLayout()
        user_layout.addWidget(self.heading_provider)
        user_layout.addWidget(heading_l)
        user_layout.addStretch(1)
        compass_layout.addLayout(current_layout)
        compass_layout.addLayout(user_layout)
        
        compass_box = QGroupBox("AUV Heading")
        compass_box.setLayout(compass_layout)
        goal_gui_layout.addWidget(compass_box)
        
        #Depth Scale
        self.depth_thermo = Qwt.QwtThermo()
        self.depth_thermo.setPipeWidth(6)
        self.depth_thermo.setRange(0, 5)
        self.depth_thermo.setFillColor(Qt.green)
        self.depth_thermo.setAlarmColor(Qt.red)
        
        #Start Status Bar
        self.status_text = QLabel('Action Server idle')
        self.statusBar().addWidget(self.status_text, 1)
        
        #Attitude Information GUI
        attitudeBox =  QGroupBox("Attitude Information")
        self.attitudePanel1 = QTextBrowser()
        self.attitudePanel1.setStyleSheet("QTextBrowser { background-color : black; color :white;}")
        self.attitudePanel2 = QTextBrowser()
        self.attitudePanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel3 = QTextBrowser()
        self.attitudePanel3.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel4 = QTextBrowser()
        self.attitudePanel4.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel5 = QTextBrowser()
        self.attitudePanel5.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        attitude_layout = QHBoxLayout()
        attitude_layout.addWidget(self.attitudePanel1)
        attitude_layout.addWidget(self.attitudePanel2)
        attitude_layout.addWidget(self.attitudePanel3)
        attitude_layout.addWidget(self.attitudePanel4)
        attitude_layout.addWidget(self.attitudePanel5)
        attitudeBox.setLayout(attitude_layout)
        #Setpoint information
        
        setpointBox = QGroupBox("Setpoint Information")
        self.setpointPanel1 = QTextBrowser()
        self.setpointPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.setpointPanel2 = QTextBrowser()
        self.setpointPanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        
        setpoint_layout = QVBoxLayout()
        setpoint_layout.addWidget(self.setpointPanel1)
        setpoint_layout.addWidget(self.setpointPanel2)
        setpointBox.setLayout(setpoint_layout)
        
        #Sensor and Actuator Information
        saBox = QGroupBox("Sensor and Actuator Information")
        self.saPanel1 = QTextBrowser()
        self.saPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.saPanel2 = QTextBrowser()
        self.saPanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.saPanel3 = QTextBrowser()
        self.saPanel3.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.saPanel4 = QTextBrowser()
        self.saPanel4.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        
        sa_layout = QHBoxLayout()
        sa_layout.addWidget(self.saPanel1)
        sa_layout.addWidget(self.saPanel2)
        sa_layout.addWidget(self.saPanel3)
        sa_layout.addWidget(self.saPanel4)
        saBox.setLayout(sa_layout)
        
        #OpenUPS Information
        oBox = QGroupBox("Battery Information")
        self.oPanel1 = QTextBrowser()
        self.oPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.oPanel2 = QTextBrowser()
        self.oPanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")

        o_layout = QHBoxLayout()
        o_layout.addWidget(self.oPanel1)
        o_layout.addWidget(self.oPanel2)
        oBox.setLayout(o_layout)
        
        display_layout = QVBoxLayout()
        display_layout.addWidget(attitudeBox)
        display_layout.addWidget(saBox)
        display_layout.addWidget(oBox)
        overall_display_layout = QHBoxLayout()
        overall_display_layout.addLayout(display_layout)
        overall_display_layout.addWidget(setpointBox) 
        
        main_layout = QHBoxLayout()
        main_Hlayout = QHBoxLayout()
        main_Vlayout = QVBoxLayout()
        main_Hlayout.addWidget(goalBox)
        main_Hlayout.addWidget(maniBox)
        main_Vlayout.addLayout(main_Hlayout)
        main_Vlayout.addLayout(overall_display_layout)
        main_layout.addLayout(main_Vlayout)
        main_layout.addWidget(self.depth_thermo)
        
        video_layout = QVBoxLayout()
        self.video_top = QLabel()
        self.video_bot = QLabel()
        video_top_l = QLabel("<b>Front Camera</b>")
        video_bot_l = QLabel("<b>Bottom Camera</b>")
        video_layout.addWidget(video_top_l)
        video_layout.addWidget(self.video_top)
        video_layout.addWidget(video_bot_l)
        video_layout.addWidget(self.video_bot)
        #video_layout.addStretch(1)
        main_layout.addLayout(video_layout)
        
        #main_layout.addLayout(compass_layout)
        self.main_frame.setLayout(main_layout)
        self.setGeometry(300, 300, 1090, 760)
        self.setWindowTitle('Bumblebee AUV Control Panel')
        self.setWindowIcon(QIcon(os.getcwd() + '/scripts/icons/field.png'))
        self.setCentralWidget(self.main_tab)
        self.heading_provider.valueChanged.connect(self.valueChanged)
        self.initImage()
        self.initAction()
        self.initSub()
        self.initService()
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.on_timer)
        self.timer.start(1000.0 / self.update_freq)
    
        if not pynotify.init("Basics"):
            sys.exit(1)
 
        n = pynotify.Notification("Welcome", "Welcome to Bumblebee AUV Systems Control Panel!")
        if not n.show():
            print "Failed to send notification"
    def on_timer(self):
        yaw = None
        depth = None
        orientation = None
        controller_setpoints = None
        controller_feedback = None
        manipulators = None
        thrusters = None
        hull_statuses = None
        rel_pos = None
        earth_pos = None
        openups = None
        temp = None
        altitude = None
        image_bot = None
        image_front = None
        image_rfront = None
        mode = None
        '''Catch if queue is Empty exceptions'''
        try:
            orientation = self.q_orientation.get(False,0)
            self.q_orientation = Queue.Queue()
        except Exception,e:
            pass
        try:
            temp = self.q_temp.get(False,0)
            self.q_temp = Queue.Queue()
        except Exception,e:
            pass   
        try:
            altitude = self.q_altitude.get(False,0)
            self.q_altitude = Queue.Queue()
        except Exception,e:
            pass
        try:
            mode = self.q_mode.get(False,0)
            self.q_mode = Queue.Queue()
        except Exception,e:
            pass
        try:
            openups = self.q_openups.get(False,0)
            self.q_openups = Queue.Queue()
        except Exception,e:
            pass
        try:
            depth = self.q_depth.get(False,0)
            self.q_depth = Queue.Queue()
        except Exception,e:
            pass
        try:
            controller_setpoints = self.q_controller.get(False,0)
            self.q_controller = Queue.Queue()
        except Exception,e:
            pass
        try:
            controller_feedback = self.q_controller_feedback.get(False, 0)
            self.q_controller_feedback = Queue.Queue()
        except Exception,e:
            pass
        try:
            thrusters = self.q_thruster.get(False,0)
            self.q_thruster = Queue.Queue()
        except Exception,e:
            pass
        try:
            manipulators = self.q_mani.get(False,0)
            self.q_mani = Queue.Queue()
        except Exception,e:
            pass
        try:
            hull_statuses = self.q_hull_status.get(False,0)
            self.q_hull_status = Queue.Queue()
        except Exception,e:
            pass
        try:
            rel_pos = self.q_rel_pos.get(False,0)
            self.q_rel_pos = Queue.Queue()
        except Exception,e:
            pass
        try:
            earth_pos = self.q_earth_pos.get(False,0)
            self.q_earth_pos = Queue.Queue()
        except Exception,e:
            pass
#        try:
#            image_rfront = self.q_image_rfront.get(False, 0)
#        except Exception,e:
#            pass
#        try:
#            image_front = self.q_image_front.get(False,0)
#        except Exception,e:
#            pass
        try:
            image_bot = self.q_image_bot
        except Exception,e:
            pass
        '''If data in queue is available store it into data'''
        if temp!= None:
            self.data['temp'] = temp.data
        if altitude!= None:
            self.data['altitude'] = altitude.data    
        if manipulators != None:
            self.data['manipulators'] = manipulators
        if orientation != None:
            self.data['pitch'] = orientation.pitch
            self.data['yaw'] = orientation.yaw
            self.data['roll'] = orientation.roll
        if hull_statuses != None:
            self.data['hull_status'] = hull_statuses
        if rel_pos != None:
            self.data['rel_pos'] = rel_pos
        if earth_pos != None:
            self.data['earth_pos'] = earth_pos
        if depth != None:
            self.data['depth'] = depth.depth
            self.data['pressure'] = depth.pressure
        if thrusters != None:
            self.data['thrusters'] = thrusters
        if openups != None:
            self.data['openups'] = openups
        if mode != None:
            self.data['mode'] = mode.data
        if controller_setpoints != None:
            self.data['heading_setpoint'] = controller_setpoints.heading_setpoint
            self.data['depth_setpoint'] = controller_setpoints.depth_setpoint
            self.data['forward_setpoint'] = controller_setpoints.forward_setpoint
            self.data['sidemove_setpoint'] = controller_setpoints.sidemove_setpoint
            
        if controller_feedback != None:
            self.data['forward_error'] =controller_feedback.feedback.forward_error
            self.data['heading_error'] =controller_feedback.feedback.heading_error
            self.data['sidemove_error'] =controller_feedback.feedback.sidemove_error
            self.data['depth_error'] =controller_feedback.feedback.depth_error
            self.data['goal_id'] = controller_feedback.status.goal_id.id
            self.data['status'] = controller_feedback.status.status
        if self.q_image_front != None:
            self.update_video_front(self.q_image_front)
        if self.vision_filter_frame.isFront == 0 and self.q_image_front!=None:
            self.update_video_rfront(self.q_image_front)
        if image_bot != None:
            self.update_video_bot(image_bot)
        if self.filter_image != None:
            self.vision_filter_frame.update_image_filterchain(self.filter_image)
            
        self.depth_thermo.setValue(round(self.data['depth'],2))    
        self.compass.setValue(int(self.data['yaw']))
        
        if self.data['mode']== 0:
            self.l_mode.setText("Default")
        elif self.data['mode'] == 1:
            self.l_mode.setText("Forward")
        elif self.data['mode'] == 2:
            self.l_mode.setText("Sidemove")
        
        self.attitudePanel1.setText("<b>YAW: " + str(round(self.data['yaw'],2)) + 
                                    "<br> PIT: " + str(round(self.data['pitch'],2)) +
                                    "<br>RLL: "+ str(round(self.data['roll'],2)) + "</b>")
        
        self.attitudePanel2.setText("<b>DEP: "+ str(round(self.data['depth'],2)) + 
                                    "<br>PRE: " + str(round(self.data['pressure']/1000,2)) + 
                                    "<br>ATT: " + str(round(self.data['altitude'],2)) + "</b>")
        
        self.attitudePanel3.setText("<b>POSX: " + str(round(self.data['earth_pos'].pose.pose.position.x,2)) + 
                                    "<br> POSY: " + str(round(self.data['earth_pos'].pose.pose.position.y,2)) + "</b>")
        
        self.attitudePanel4.setText("<b>RPOSX: " + str(round(self.data['rel_pos'].pose.pose.position.x,2)) + 
                                    "<br>RPOSY: " + str(round(self.data['rel_pos'].pose.pose.position.y,2)) + 
                                    "<br>RPOSZ : " + str(round(self.data['rel_pos'].pose.pose.position.z,2)) + "</b>")
        
        self.attitudePanel5.setText("<b>VELX: " + str(round(self.data['rel_pos'].twist.twist.linear.x,2)) + 
                                    "<br> VELY: " + str(round(self.data['rel_pos'].twist.twist.linear.y,2)) +
                                    "<br>VELZ: " + str(round(self.data['rel_pos'].twist.twist.linear.z,2)) + "</b>")
        
        self.saPanel1.setText("<b>THR1: " + str(self.data['thrusters'].speed1) + 
                              "<br> THR2: " + str(self.data['thrusters'].speed2) +
                              "<br> THR3: " + str(self.data['thrusters'].speed3) +
                              "<br> THR4: " + str(self.data['thrusters'].speed4) + "</b>")
        self.saPanel2.setText("<b>THR5: " + str(self.data['thrusters'].speed5) +
                              "<br> THR6: " + str(self.data['thrusters'].speed6) +
                              "<br> THR7: " + str(self.data['thrusters'].speed7) +
                              "<br> THR8: " + str(self.data['thrusters'].speed8) + "</b>")
        
        mani_name = ["","","","","","",""]
        if self.data['manipulators'].servo1:
            mani_name[0] = "OPEN"
        else:
            mani_name[0] = "CLOSED"
        
        if self.data['manipulators'].servo2:
            mani_name[1] = "OPEN"
        else:
            mani_name[1] = "CLOSED"
        
        if self.data['manipulators'].servo3:
            mani_name[2] = "FIRED"
        else:
            mani_name[2] = "NONE"
        
        if self.data['manipulators'].servo4:
            mani_name[3] = "FIRED"
        else:
            mani_name[3] = "NONE"
            
        if self.data['manipulators'].servo5:
            mani_name[4] = "CLOSED"
        else:
            mani_name[4] = "OPENED"
        
        self.saPanel3.setText("<b>LD: " + mani_name[0] + 
                              "<br> RD: " + mani_name[1] +
                              "<br> LT: " + mani_name[2] + 
                              "<br> RT: " + mani_name[3] +
                              "<br> GA: " + mani_name[4] +
                              #"<br> LACT: " + str(self.data['manipulators'].servo6) + 
                              #"<br> RACT: " + str(self.data['manipulators'].servo7) +
                              "</b>")
    
        self.saPanel4.setText("<b>TMP0: " + str(round(self.data['temp'],2)) + 
                              "<br> W1: " + str(self.data['hull_status'].WaterDetA) +
                              "<br> W2: " + str(self.data['hull_status'].WaterDetB) +
                              "<br> W3: " + str(self.data['hull_status'].WaterDetC) + "</b>")
        
        if (self.data['hull_status'].WaterDetA or self.data['hull_status'].WaterDetB or self.data['hull_status'].WaterDetC) and not self.isLeak:
            n = pynotify.Notification("Leak Alert", "Water ingression in vehicle detected.\n Recover Vehicle NOW!!")
            if not n.show():
                print "Failed to send notification"
            self.isLeak = True
        else:
            self.isLeak = False
        
        batt_state = list()
        if self.data['openups'].charge1== -1:
            batt_state.append("DISCON")
        elif self.data['openups'].charge1== -2:
            batt_state.append("OFF")
        elif self.data['openups'].charge1== -3:
            batt_state.append("LOADING")
        else:
            batt_state.append(self.data['openups'].charge1)
        
        if self.data['openups'].charge2== -1:
            batt_state.append("DISCON")
        elif self.data['openups'].charge2== -2:
            batt_state.append("OFF")
        elif self.data['openups'].charge2== -3:
            batt_state.append("LOADING")
        else:
            batt_state.append(self.data['openups'].charge2) 
        
        
        if self.data['openups'].charge1 < 10 and self.data['openups'].charge1 > -1 and self.isAlert[0] == False:
            self.showDialog(1)
            self.isAlert[0] = True
        if self.data['openups'].charge2 < 10 and self.data['openups'].charge2 > -1 and self.isAlert[1] == False:
            self.showDialog(2)
            self.isAlert[1] = True
          
        if self.data['openups'].charge1 > 10:
            self.isAlert[0] = False
        if self.data['openups'].charge2 > 10:
            self.isAlert[1] = False

        
        self.oPanel1.setText("<b>OUPS1: " + str(batt_state[0]) + 
                              "%<br> CUR1: " + str(round(self.data['openups'].current1,2)) +
                              "<br> VOLT1: " + str(round(self.data['openups'].voltage1,2))+ "</b>")
        self.oPanel2.setText("<b>OUPS2: " + str(batt_state[1]) + 
                              "%<br> CUR2: " + str(round(self.data['openups'].current2,2)) +
                              "<br> VOLT2: " + str(round(self.data['openups'].voltage2,2))+ "</b>")
        
        self.setpointPanel1.setText("<b>HDG: " + str(round(self.data['heading_setpoint'],2)) + "<br> FWD: " + str(round(self.data['forward_setpoint'],2)) + 
                                    "<br>SIDE: "+ str(round(self.data['sidemove_setpoint'],2)) + "<br>DEP: "+ str(round(self.data ['depth_setpoint'],2)) + "</b>")
        
        goal_string = self.data['goal_id'].partition('-')
        self.setpointPanel2.setText("<b>ID: " + goal_string[0] +
                                    "<br>ST: " + self.get_status(self.data['status']) + 
                                    "<br>HDG ERR: " + str(round(self.data['heading_error'],2)) + 
                                    "<br> FWD ERR: " + str(round(self.data['forward_error'],2)) + 
                                    "<br>SIDE ERR: "+ str(round(self.data['sidemove_error'],2)) + 
                                    "<br>DEP ERR: "+ str(round(self.data ['depth_error'],2)) + "</b>")
        
    def showDialog(self,ups):
        #QMessageBox.about(self,"Battery Low",)
        n = pynotify.Notification("Battery Low", "OpenUPS " + str(ups) + " is low on battery.\n Replace now!")
        if not n.show():
            print "Failed to send notification"
        
    def initService(self):
        rospy.wait_for_service('set_controller_srv')
        rospy.loginfo("set_controller Service ready.")
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        
        #rospy.wait_for_service('locomotion_mode_srv')
        rospy.loginfo("Locomotion Mode Service ready.")
        self.locomotion_mode_request = rospy.ServiceProxy('locomotion_mode_srv',locomotion_mode)
        
    def initImage(self):
        self.bridge = CvBridge()
        frontcam_sub = rospy.Subscriber(rospy.get_param('~front',"/front_camera/camera/image_rect_color/image_raw"),Image, self.front_callback)
        #frontcam_sub = rospy.Subscriber(rospy.get_param('~front_right',"/stereo_camera/right/image_rect_color_opt"),Image, self.front_rcallback)
        botcam_sub = rospy.Subscriber(rospy.get_param('~bottom',"/bot_cam/camera/image_rect_color/image_raw"),Image, self.bottom_callback)
        filter_sub = rospy.Subscriber(rospy.get_param('~filter',"/Vision/image_filter_opt"),Image, self.filter_callback)
        
    def initSub(self):
        thruster_sub = rospy.Subscriber("/thruster_speed",thruster, self.thruster_callback)
        depth_sub = rospy.Subscriber("/depth", depth ,self.depth_callback)
        orientation_sub = rospy.Subscriber("/euler", compass_data ,self.orientation_callback)
        position_sub = rospy.Subscriber("/WH_DVL_data", Odometry ,self.rel_pos_callback)
        controller_sub = rospy.Subscriber("/controller_points",controller,self.controller_callback)
        self.mani_pub = rospy.Publisher("/manipulator",manipulator)
        self.mani_sub = rospy.Subscriber("/manipulator",manipulator,self.manipulators_callback)
        self.earth_sub = rospy.Subscriber("/earth_odom",Odometry,self.earth_pos_callback)
        feedback_sub = rospy.Subscriber("/LocomotionServer/feedback",ControllerActionFeedback,self.controller_feedback_callback)
        self.hull_status_sub = rospy.Subscriber("/hull_status", hull_status, self.hull_status_callback)
        openups_sub = rospy.Subscriber("/openups_stats",openups_stats,self.openups_callback)
        temp_sub = rospy.Subscriber("/AHRS8_Temp",Float32,self.temp_callback)
        altitude_sub =  rospy.Subscriber("/altitude",Float32,self.altitude_callback)
        mode_sub = rospy.Subscriber("/locomotion_mode",Int8,self.mode_callback)
    def get_status(self,val):
        if val == -1:
            return "NONE"
        if val == 0:
            return "PENDING"
        if val == 1:
            return "ACTIVE"
        if val == 2:
            return "PREEMPTED"
        if val == 3:
            return "SUCCEEDED"
        if val == 4:
            return "ABORTED"
        if val == 5:
            return "REJECTED"
        if val == 6:
            return "PREEMPTING"
        if val == 7:
            return "RECALLING"
        if val == 8:
            return "RECALLED"
        if val == 9:
            return "LOST"

    def homeBtnHandler(self):
        movebaseGoal = MoveBaseGoal()
        x,y,z,w = quaternion_from_euler(0,0,(360 -(self.data['yaw'] + 180) * (pi/180))) #input must be radians
        resp = self.set_controller_request(True, True, True, True, False, False,True)
        #Execute Nav
        movebaseGoal.target_pose.header.frame_id = 'map'
        movebaseGoal.target_pose.header.stamp = rospy.Time.now()
        movebaseGoal.target_pose.pose.position.x = 0
        movebaseGoal.target_pose.pose.position.y = 0 
        movebaseGoal.target_pose.pose.orientation.x = 0
        movebaseGoal.target_pose.pose.orientation.y = 0
        movebaseGoal.target_pose.pose.orientation.z = z
        movebaseGoal.target_pose.pose.orientation.w = w
        self.movebase_client.send_goal(movebaseGoal, self.movebase_done_cb)
        #movebase_client.wait_for_result(rospy.Duration(self.nav_timeout,0))
    def hoverBtnHandler(self):
        resp = self.set_controller_request(True, True, True, True, True, False,False)
        goal = ControllerGoal
        goal.depth_setpoint = self.data['depth']
        goal.sidemove_setpoint = 0
        goal.heading_setpoint = self.data['yaw']
        goal.forward_setpoint = 0
        self.client.send_goal(goal, self.done_cb)
        
    def surfaceBtnHandler(self):
        resp = self.set_controller_request(True, True, True, True, False, False,False)
        goal = ControllerGoal
        goal.depth_setpoint = 0
        goal.sidemove_setpoint = 0
        goal.heading_setpoint = self.data['yaw']
        goal.forward_setpoint = 0
        self.client.send_goal(goal, self.done_cb)
        
    def modeBtnHandler(self):
        if(self.counter == 0):
            self.modeButton.setText("Forward")
            #Enable Forward Mode
            resp = self.locomotion_mode_request(True,False)
            self.counter = self.counter + 1
        elif(self.counter == 1):
            resp = self.locomotion_mode_request(False,True)
            self.modeButton.setText("Sidemove")
            #Enable Sidemove Mode
            self.counter = self.counter + 1
        elif(self.counter == 2):
            resp = self.locomotion_mode_request(False,False)
            self.modeButton.setText("Default")
            #Enable Default Mode
            
            self.counter = 0

    def startBtnHandler(self):
        self.status_text.setText("Action Client executing goal...")
        resp = self.set_controller_request(True, True, True, True, True, False,False)
        goal = ControllerGoal
        if self.rel_depth_chkbox.checkState():
            goal.depth_setpoint = self.data['depth'] + float(self.depth_box.text())
        else:    
            goal.depth_setpoint = float(self.depth_box.text())
        goal.sidemove_setpoint = float(self.sidemove_box.text())
        if self.rel_heading_chkbox.checkState():
            goal.heading_setpoint = (self.data['yaw'] + float(self.heading_box.text())) % 360
        else:    
            goal.heading_setpoint = float(self.heading_box.text())
        goal.forward_setpoint = float(self.forward_box.text())
        self.client.send_goal(goal, self.done_cb)
        
    def fireBtnHandler(self):
        if(self.isArmed):
            #_manipulator = manipulator()
            
            if(self.check1.checkState()):
                servo1 = 1
            else:
                servo1 = 0
            if(self.check2.checkState()):
                servo2 = 1
            else:
                servo2 = 0
            if(self.check3.checkState()):
                servo3 = 1
            else:
                servo3 = 0
            if(self.check4.checkState()):
                servo4 = 1
            else:
                servo4 = 0
            if(self.check5.checkState()):
                servo5 = 1
            else:
                servo5 = 0
            if(self.check6.checkState()):
                servo6 = 1
            else:
                servo6 = 0
            if(self.check7.checkState()):
                servo7 = 1
            else:
                servo7 = 0
            
            self.mani_pub.publish(servo1,servo2,servo3,servo4,servo5,servo6,servo7)
        
    def armBtnHandler(self):
        if(self.isArmed):
            self.isArmed = False
            self.armButton.setText("NOT ARMED")
        else:
            self.armButton.setText("ARMED")
            self.isArmed = True
            
    def done_cb(self,status,result):
        self.status_text.setText("Action Client completed goal!")
        #resp = self.set_controller_request(False, False, False, False, False, True)
    
    def movebase_done_cb(self,status,result):
        self.status_text.setText("Move Base Client completed goal!")
    def endBtnHandler(self):
        self.client.cancel_all_goals()
        self.movebase_client.cancel_all_goals()
        self.status_text.setText("Action Client ended goal.")
        #resp = self.set_controller_request(False, False, False, False, False, True, False)
    def initAction(self):
        self.client = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        rospy.loginfo("Waiting for Action Server to connect.")
        self.status_text.setText("Waiting for Action Server to connect.")
        #self.client.wait_for_server()
        rospy.loginfo("Action Server connected.")
        self.status_text.setText("Action Server connected.")
        self.movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #self.movebase_client.wait_for_server()
        rospy.loginfo("Mission connected to MovebaseServer")
    def valueChanged(self,value):
        self.heading_box.setText(str(value))
    def make_data_box(self, name):
        label = QLabel(name)
        qle = QLineEdit()
        layout = QHBoxLayout()
        #qle.setEnabled(False)
        layout.addWidget(label)
        layout.addWidget(qle)
        layout.addStretch(1)
        qle.setFrame(False)
        
        return (label, qle, layout)
    
    def make_data_chkbox(self, name):
        label = QLabel(name)
        qle = QCheckBox()
        layout = QHBoxLayout()
        #qle.setEnabled(False)
        layout.addWidget(label)
        layout.addWidget(qle)
        layout.addStretch(1)
        
        return (label, qle, layout)
    
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image trackbarnameto Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype
    
    def update_video_front(self,image):
        #convert numpy mat to pixmap image
        cvRGBImg_front = cv2.cvtColor(self.rosimg2cv(image), cv2.cv.CV_BGR2RGB)
        bbLock = threading.Lock()
        try:
            bbLock.acquire()
            qimg = QImage(cvRGBImg_front.data,cvRGBImg_front.shape[1], cvRGBImg_front.shape[0], QImage.Format_RGB888)
        finally:
            bbLock.release()
        qpm = QPixmap.fromImage(qimg)
        self.video_top.setPixmap(qpm.scaledToHeight(250))
    
    def update_video_rfront(self,image):
        bbLock = threading.Lock()
#        try:
#            bbLock.acquire()
#            qimg = QImage(cvRGBImg_top.data,cvRGBImg_top.shape[1], cvRGBImg_top.shape[0], QImage.Format_RGB888)
#        finally:
#            bbLock.release()
        self.vision_filter_frame.update_image_visual(image)
        self.vision_filter_frame.update_image_filter(image)
        
    def update_video_bot(self,image):
        cvRGBImg_bot = cv2.cvtColor(self.rosimg2cv(image), cv2.cv.CV_BGR2RGB)
        ####
        qimg = QImage(cvRGBImg_bot.data,cvRGBImg_bot.shape[1], cvRGBImg_bot.shape[0], QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)
        self.video_bot.setPixmap(qpm.scaledToHeight(250))
        
        if self.vision_filter_frame.isFront == 1:
            self.vision_filter_frame.update_image_visual(image)
            self.vision_filter_frame.update_image_filter(image)
    
    def front_rcallback(self,image):
        try:
            self.q_image_rfront = image
        except CvBridgeError, e:
            print e
            
    def front_callback(self,image):
        try:
            self.q_image_front = image
        except CvBridgeError, e:
            print e
        
    def bottom_callback(self,image):
        try:
            self.q_image_bot = image
        except CvBridgeError, e:
            print e
            
    def mode_callback(self,mode):
        self.q_mode.put(mode)
    def thruster_callback(self,thruster):
        self.q_thruster.put(thruster)
    def orientation_callback(self,msg):
        self.q_orientation.put(msg)
    
    def altitude_callback(self,altitude):
        self.q_altitude.put(altitude)
        
    def depth_callback(self,depth):
        self.q_depth.put(depth)

    def controller_callback(self,controller):
        self.q_controller.put(controller)
    
    def rel_pos_callback(self,pos):
        self.q_rel_pos.put(pos)
        
    def earth_pos_callback(self,earth):
        self.q_earth_pos.put(earth)
    
    def filter_callback(self,image):
        self.filter_image = image
    def hull_status_callback(self,hull):
        self.q_hull_status.put(hull)
        
    def controller_feedback_callback(self,feedback):
        self.q_controller_feedback.put(feedback)
        
    def openups_callback(self,stats):
        self.q_openups.put(stats)
        
    def temp_callback(self,temp):
        self.q_temp.put(temp)
        
    def manipulators_callback(self,mani):
        self.q_mani.put(mani)
        
if __name__ == "__main__":
    rospy.init_node('Telemetry', anonymous=True)
    app = QApplication(sys.argv)
    form = AUV_gui()
    form.show()
    app.exec_()
