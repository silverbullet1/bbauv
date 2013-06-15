#!/usr/bin/env python2
import roslib; roslib.load_manifest('auv_gui')
import rospy

from bbauv_msgs.srv import *
from bbauv_msgs.msg import *
from nav_msgs.msg import Odometry

import actionlib

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt
import Queue

class AUV_gui(QMainWindow):
    main_frame = None
    compass = None 
    heading_provider = None
    depth_thermo = None
    client = None
    yaw = 0
    depth = 0
    pos_x = 0
    pos_y = 0
    update_freq = 5
    q_yaw = Queue.Queue()
    q_depth = Queue.Queue() 
    q_pos_x = Queue.Queue()
    q_pos_y = Queue.Queue()
    
    def __init__(self, parent=None):
        super(AUV_gui, self).__init__(parent)
        self.main_frame = QWidget()
        
        goalBox =  QGroupBox("Goal Setter")
        depth_l , self.depth_box, layout4 = self.make_data_box("Depth:")
        sidemove_l, self.sidemove_box,layout2 = self.make_data_box("Sidemove:")
        forward_l, self.forward_box,layout1 = self.make_data_box("Forward:")
        heading_l, self.heading_box,layout3 = self.make_data_box("Heading:")
        
        goal_layout = QVBoxLayout()
        goal_layout.addLayout(layout1)
        goal_layout.addLayout(layout2)
        goal_layout.addLayout(layout3)
        goal_layout.addLayout(layout4)
        okButton = QPushButton("Start Goal")
        cancelButton = QPushButton("End Goal")
        disableButton = QPushButton("Disable PIDs")
        hoverButton = QPushButton("Hover")
        surfaceButton = QPushButton("Surface")
        okButton.clicked.connect(self.startBtnHandler)
        cancelButton.clicked.connect(self.endBtnHandler) 
        disableButton.clicked.connect(self.disableBtnHandler)
        surfaceButton.clicked.connect(self.surfaceBtnHandler)
        hoverButton.clicked.connect(self.hoverBtnHandler)
        hbox = QHBoxLayout()
       # hbox.addStretch(1)
        hbox.addWidget(okButton)
        hbox.addWidget(cancelButton)
        hbox.addWidget(disableButton)
        hbox2 = QHBoxLayout()
       # hbox.addStretch(1)
        hbox2.addWidget(hoverButton)
        hbox2.addWidget(surfaceButton)
        
        goal_layout.addLayout(hbox)
        goal_layout.addLayout(hbox2)
        goalBox.setLayout(goal_layout)
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
        compass_l = QLabel("Heading")
        
        compass_layout = QVBoxLayout()
        compass_layout.addWidget(compass_l)
        compass_layout.addWidget(self.compass)
        compass_layout.addWidget(self.heading_provider)
        
        #Depth Scale
        self.depth_thermo = Qwt.QwtThermo()
        self.depth_thermo.setPipeWidth(6)
        self.depth_thermo.setRange(0, 5)
        self.depth_thermo.setFillColor(Qt.green)
        self.depth_thermo.setAlarmColor(Qt.red)
        
        #Start Status Bar
        self.status_text = QLabel('Action Server idle')
        self.statusBar().addWidget(self.status_text, 1)
        
        #Current Status
        self.depth_disp_l = QLabel("Depth: 0")
        self.heading_disp_l = QLabel("Heading: 0")
        self.positionx_disp_l = QLabel("Position x: 0")
        self.positiony_disp_l = QLabel("Position y: 0")
        
        display_layout = QVBoxLayout()
        display_layout.addWidget(self.depth_disp_l)
        display_layout.addWidget(self.heading_disp_l)
        display_layout.addWidget(self.positionx_disp_l)
        display_layout.addWidget(self.positiony_disp_l)
        
        main_layout = QHBoxLayout()
        main_layout.addWidget(goalBox)
        main_layout.addLayout(display_layout)
        main_layout.addWidget(self.depth_thermo)
        main_layout.addLayout(compass_layout)
        self.main_frame.setLayout(main_layout)
        self.setGeometry(300, 300, 600, 400)
        self.setWindowTitle('AUV Locomotion Interface')  
        self.setCentralWidget(self.main_frame)
        self.heading_provider.valueChanged.connect(self.valueChanged)
        self.initAction()
        self.initSub()
        self.initService()
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.on_timer)
        self.timer.start(1000.0 / self.update_freq)
        
    def on_timer(self):
        yaw = None
        depth = None
        pos_x = None
        pos_y = None
        try:
            yaw = self.q_yaw.get(False,0)
            depth = self.q_depth.get(False,0)
            pos_x = self.q_pos_x.get(False,0)
            pos_y = self.q_pos_y.get(False,0)
        except Exception,e:
            pass
        if yaw != None:
            self.compass.setValue(int(yaw))
            self.heading_disp_l.setText("Heading: " + str(round(yaw,2)))
        if pos_x != None:
            self.positionx_disp_l.setText("Forward Position: " + str(round(pos_x,2)))
            self.positiony_disp_l.setText("Sidemove Position: " + str(round(pos_y,2)))
        if depth != None:
            self.depth_disp_l.setText("Depth: " + str(round(depth,2)))
            self.depth_thermo.setValue(round(depth,2))
    
    def initService(self):
        rospy.wait_for_service('set_controller_srv')
        self.status_text.setText("Service ready.")
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
  
    def initSub(self):
        depth_sub = rospy.Subscriber("/depth", bbauv_msgs.msg.depth ,self.depth_callback)
        orientation_sub = rospy.Subscriber("/euler", bbauv_msgs.msg.compass_data ,self.orientation_callback)
        position_sub = rospy.Subscriber("/WH_DVL_data", Odometry ,self.position_callback)
    
    def hoverBtnHandler(self):
        resp = self.set_controller_request(True, True, True, True, False, False,False)
        goal = bbauv_msgs.msg.ControllerGoal
        goal.depth_setpoint = self.depth
        goal.sidemove_setpoint = 0
        goal.heading_setpoint = self.yaw
        goal.forward_setpoint = 0
        self.client.send_goal(goal, self.done_cb)
        
    def surfaceBtnHandler(self):
        resp = self.set_controller_request(True, True, True, True, False, False,False)
        goal = bbauv_msgs.msg.ControllerGoal
        goal.depth_setpoint = 0
        goal.sidemove_setpoint = 0
        goal.heading_setpoint = self.yaw
        goal.forward_setpoint = 0
        self.client.send_goal(goal, self.done_cb)
        
    def disableBtnHandler(self):
        resp = self.set_controller_request(False, False, False, False, False, True, False)
        print "resp:" + str(resp)
    def startBtnHandler(self):
        self.status_text.setText("Action Client executing goal...")
        resp = self.set_controller_request(True, True, True, True, False, False,False)
        print "resp:" + str(resp)
        goal = bbauv_msgs.msg.ControllerGoal
        goal.depth_setpoint = float(self.depth_box.text())
        goal.sidemove_setpoint = float(self.sidemove_box.text())
        goal.heading_setpoint = float(self.heading_box.text())
        goal.forward_setpoint = float(self.forward_box.text())
        self.client.send_goal(goal, self.done_cb)
    
    def done_cb(self,status,result):
        self.status_text.setText("Action Client completed goal!")
        #resp = self.set_controller_request(False, False, False, False, False, True, False)
        
    def endBtnHandler(self):
        self.client.cancel_all_goals()
        self.status_text.setText("Action Client ended goal.")
        #resp = self.set_controller_request(False, False, False, False, False, True, False)
    def initAction(self):
        self.client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        rospy.loginfo("Waiting for Action Server to connect.")
        self.status_text.setText("Waiting for Action Server to connect.")
        self.client.wait_for_server()
        self.status_text.setText("Action Server connected.")
    
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
    
    def orientation_callback(self,msg):
        self.q_yaw.put(msg.yaw)
        #self.compass.setValue(int(msg.yaw))
        #self.heading_disp_l.setText("Heading: " + str(round(msg.yaw,2)))
        #self.heading_box = self.heading_provider.getValue()
    def position_callback(self,pos):
        self.q_pos_x.put(pos.pose.pose.position.x)
        self.q_pos_y.put(pos.pose.pose.position.y)
        #self.positionx_disp_l.setText("Forward Position: " + str(round(pos.pose.pose.position.x,2)))
        #self.positiony_disp_l.setText("Sidemove Position: " + str(round(pos.pose.pose.position.y,2)))
    def depth_callback(self,depth):
        self.q_depth.put(depth.depth)
        #self.depth_disp_l.setText("Depth: " + str(round(depth.depth,2)))
        #form.compass.setValue(depth.depth)
        #self.depth_thermo.setValue(round(depth.depth,2))
        
if __name__ == "__main__":
    rospy.init_node('AUV_gui', anonymous=True)
    app = QApplication(sys.argv)
    form = AUV_gui()
    form.show()
    
    app.exec_()
    
    
    
