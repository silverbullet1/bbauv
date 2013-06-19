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
from bbauv_msgs.msg._thruster import thruster

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
    isArmed = False
    update_freq = 50
    q_orientation = Queue.Queue()
    q_depth = Queue.Queue() 
    q_earth_pos = Queue.Queue()
    q_rel_pos = Queue.Queue()
    q_controller = Queue.Queue()
    q_hull_status = Queue.Queue()
    q_manipulators = Queue.Queue()
    q_controller_feedback = Queue.Queue()
    q_thruster = Queue.Queue()
    data = {'yaw': 0, 'pitch' : 0,'roll':0, 'depth': 0, 'attitude':0,
            'pressure':0,'forward_setpoint':0,'sidemove_setpoint':0,
            'heading_setpoint':0,'depth_setpoint':0,'altitude':0,'heading_error':0,
            'forward_error':0,'sidemove_error':0,'depth_error':0,'goal_id':"None",'thrusters':thruster(),
            'hull_status':hull_status(),'status':-1,'earth_pos':Odometry(),'rel_pos':Odometry(),'manipulators':manipulator()}
    
    def __init__(self, parent=None):
        super(AUV_gui, self).__init__(parent)
        self.main_frame = QWidget()
        
        goalBox =  QGroupBox("Goal Setter")
        depth_l , self.depth_box, layout4 = self.make_data_box("Depth:       ")
        sidemove_l, self.sidemove_box,layout2 = self.make_data_box("Sidemove:")
        forward_l, self.forward_box,layout1 = self.make_data_box("Forward:   ")
        heading_l, self.heading_box,layout3 = self.make_data_box("Heading:   ")
        
        goal_layout = QVBoxLayout()
        goal_layout.addLayout(layout1)
        goal_layout.addLayout(layout2)
        goal_layout.addLayout(layout3)
        goal_layout.addLayout(layout4)
        
        # Buttons Layout
        okButton = QPushButton("Start Goal")
        cancelButton = QPushButton("End Goal")
        disableButton = QPushButton("Disable PIDs")
        hoverButton = QPushButton("Hover")
        surfaceButton = QPushButton("Surface")
        homeButton = QPushButton("Home Base")
        self.armButton = QCommandLinkButton("NOT ARMED")
        fireButton = QPushButton("Fire")
        self.check1 = QCheckBox("Left Dropper")
        self.check2 = QCheckBox("Right Dropper")
        self.check3 = QCheckBox("Left Torpedo")
        self.check4 = QCheckBox("Right Torpedo")
        self.check5 = QCheckBox("Grabber Actuator")
        self.check6 = QCheckBox("Linear Actuator")
        self.check7 = QCheckBox("Rotary Actuator")
        
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
        disableButton.clicked.connect(self.disableBtnHandler)
        surfaceButton.clicked.connect(self.surfaceBtnHandler)
        hoverButton.clicked.connect(self.hoverBtnHandler)
        homeButton.clicked.connect(self.homeBtnHandler)
        vbox = QVBoxLayout()
       # hbox.addStretch(1)
        vbox.addWidget(okButton)
        vbox.addWidget(cancelButton)
        vbox.addWidget(disableButton)
        vbox2 = QVBoxLayout()
       # hbox.addStretch(1)
        vbox2.addWidget(hoverButton)
        vbox2.addWidget(surfaceButton)
        vbox2.addWidget(homeButton)
        goalBox_layout = QVBoxLayout()
        goalBox_layout.addLayout(goal_layout)
        goalBtn_layout = QHBoxLayout()
        goalBtn_layout.addLayout(vbox)
        goalBtn_layout.addLayout(vbox2)
        goalBox_layout.addStretch(1)
        goalBox_layout.addLayout(goalBtn_layout)
        vbox.addStretch(1)
        vbox2.addStretch(1)
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
        self.attitudePanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel2 = QTextBrowser()
        self.attitudePanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel3 = QTextBrowser()
        self.attitudePanel3.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        attitude_layout = QHBoxLayout()
        attitude_layout.addWidget(self.attitudePanel1)
        attitude_layout.addWidget(self.attitudePanel2)
        attitude_layout.addWidget(self.attitudePanel3)
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
        
        
        sa_layout = QHBoxLayout()
        sa_layout.addWidget(self.saPanel1)
        sa_layout.addWidget(self.saPanel2)
        sa_layout.addWidget(self.saPanel3)
        saBox.setLayout(sa_layout)
        
        display_layout = QVBoxLayout()
        display_layout.addWidget(attitudeBox)
        display_layout.addWidget(saBox)
        overall_display_layout = QHBoxLayout()
        overall_display_layout.addLayout(display_layout)
        overall_display_layout.addWidget(setpointBox) 
        
        main_layout = QHBoxLayout()
        main_Hlayout = QHBoxLayout()
        main_Vlayout = QVBoxLayout()
        main_Hlayout.addWidget(goalBox)
        main_Hlayout.addWidget(maniBox)
        main_Hlayout.addWidget(compass_box)
        
        #main_Vlayout.addWidget(goalBox)
        #main_Vlayout.addWidget(compass_box)
        main_Vlayout.addLayout(main_Hlayout)
        main_Vlayout.addLayout(overall_display_layout)
        main_layout.addLayout(main_Vlayout)
        main_layout.addWidget(self.depth_thermo)
        #main_layout.addLayout(compass_layout)
        self.main_frame.setLayout(main_layout)
        self.setGeometry(300, 300, 800, 670)
        self.setWindowTitle('Bumblebee AUV Control Panel')  
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
        orientation = None
        controller_setpoints = None
        controller_feedback = None
        manipulators = None
        thrusters = None
        hull_statuses = None
        rel_pos = None
        earth_pos = None
        '''Catch if queue is Empty exceptions'''
        try:
            orientation = self.q_orientation.get(False,0)
        except Exception,e:
            pass
        try:
            depth = self.q_depth.get(False,0)
        except Exception,e:
            pass
        try:
            controller_setpoints = self.q_controller.get(False,0)
        except Exception,e:
            pass
        try:
            controller_feedback = self.q_controller_feedback.get(False, 0)
        except Exception,e:
            pass
        try:
            thrusters = self.q_thruster.get(False,0)
        except Exception,e:
            pass
        try:
            manipulators = self.q_manipulators.get(False,0)
        except Exception,e:
            pass
        try:
            hull_statuses = self.q_hull_status.get(False,0)
        except Exception,e:
            pass
        try:
            rel_pos = self.q_rel_pos.get(False,0)
        except Exception,e:
            pass
        try:
            earth_pos = self.q_earth_pos.get(False,0)
        except Exception,e:
            pass
        
        '''If data in queue is available store it into data'''
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
        self.depth_thermo.setValue(round(self.data['depth'],2))    
        self.compass.setValue(int(self.data['yaw']))
        self.attitudePanel1.setText("<b>YAW: " + str(round(self.data['yaw'],2)) + 
                                    "<br> PIT: " + str(round(self.data['pitch'],2)) +
                                    "<br>RLL: "+ str(round(self.data['roll'],2)) + 
                                    "<br>DEP: "+ str(round(self.data['depth'],2)) + 
                                    "<br>PRE: " + str(round(self.data['pressure'],2)) + 
                                    "<br>ATT: " + str(round(self.data['attitude'],2)) + "</b>")

        self.attitudePanel2.setText("<b>POSX: " + str(round(self.data['earth_pos'].pose.pose.position.x,2)) + 
                                    "<br> POSY: " + str(round(self.data['earth_pos'].pose.pose.position.y,2)) +
                                    "<br>RPOSX: " + str(round(self.data['rel_pos'].pose.pose.position.x,2)) + 
                                    "<br>RPOSY: " + str(round(self.data['rel_pos'].pose.pose.position.y,2)) + 
                                    "<br>RPOSZ : " + str(round(self.data['rel_pos'].pose.pose.position.z,2)) + "</b>")
        
        self.attitudePanel3.setText("<b>VELX: " + str(round(self.data['rel_pos'].twist.twist.linear.x,2)) + 
                                    "<br> VELY: " + str(round(self.data['rel_pos'].twist.twist.linear.y,2)) +
                                    "<br>VELZ: " + str(round(self.data['rel_pos'].twist.twist.linear.z,2)) + "</b>")
        
        self.saPanel1.setText("<b>THR1: " + str(self.data['thrusters'].speed1) + 
                              "<br> THR2: " + str(self.data['thrusters'].speed2) +
                              "<br> THR3: " + str(self.data['thrusters'].speed3) + 
                              "<br> THR4: " + str(self.data['thrusters'].speed4) +
                              "<br> THR5: " + str(self.data['thrusters'].speed5) +
                              "<br> THR6: " + str(self.data['thrusters'].speed6) + "</b>")
        self.saPanel2.setText("<b>LDROP: " + str(self.data['manipulators'].servo1) + 
                              "<br> RDROP: " + str(self.data['manipulators'].servo2) +
                              "<br> LTOR: " + str(self.data['manipulators'].servo3) + 
                              "<br> RTOR: " + str(self.data['manipulators'].servo4) +
                              "<br> GACT: " + str(self.data['manipulators'].servo5) +
                              "<br> LACT: " + str(self.data['manipulators'].servo6) + 
                              "<br> RACT: " + str(self.data['manipulators'].servo7) + "</b>")
      
        
        self.saPanel3.setText("<b>TMP0: " + str(self.data['hull_status'].Temp0) + 
                              "<br> TMP1: " + str(self.data['hull_status'].Temp1) +
                              "<br> TMP2: " + str(self.data['hull_status'].Temp2) + 
                              "<br> W1: " + str(self.data['hull_status'].WaterDetA) +
                              "<br> W2: " + str(self.data['hull_status'].WaterDetB) +
                              "<br> W3: " + str(self.data['hull_status'].WaterDetC) + "</b>")
        
        self.setpointPanel1.setText("<b>HDG: " + str(round(self.data['heading_setpoint'],2)) + "<br> FWD: " + str(round(self.data['forward_setpoint'],2)) + 
                                    "<br>SIDE: "+ str(round(self.data['sidemove_setpoint'],2)) + "<br>DEP: "+ str(round(self.data ['depth_setpoint'],2)) + "</b>")
        
        self.setpointPanel2.setText("<b>ID: " + self.data['goal_id'] +
                                    "<br>ST: " + self.get_status(self.data['status']) + 
                                    "<br>HDG ERR: " + str(round(self.data['heading_error'],2)) + 
                                    "<br> FWD ERR: " + str(round(self.data['forward_error'],2)) + 
                                    "<br>SIDE ERR: "+ str(round(self.data['sidemove_error'],2)) + 
                                    "<br>DEP ERR: "+ str(round(self.data ['depth_error'],2)) + "</b>")
        
    def initService(self):
        rospy.wait_for_service('set_controller_srv')
        self.status_text.setText("Service ready.")
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
  
    def initSub(self):
        thruster_sub = rospy.Subscriber("/thruster_speed",thruster, self.thruster_callback)
        depth_sub = rospy.Subscriber("/depth", bbauv_msgs.msg.depth ,self.depth_callback)
        orientation_sub = rospy.Subscriber("/euler", bbauv_msgs.msg.compass_data ,self.orientation_callback)
        position_sub = rospy.Subscriber("/WH_DVL_data", Odometry ,self.rel_pos_callback)
        controller_sub = rospy.Subscriber("/controller_points",controller,self.controller_callback)
        self.mani_pub = rospy.Publisher("/manipulators",manipulator)
        self.earth_sub = rospy.Subscriber("/earth_odometry",Odometry,self.earth_pos_callback)
        feedback_sub = rospy.Subscriber("/LocomotionServer/feedback",ControllerActionFeedback,self.controller_feedback_callback)
        self.hull_status_sub = rospy.Subscriber("/hull_status", hull_status, self.hull_status_callback)
    
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
        pass
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
    
    def fireBtnHandler(self):
        if(self.isArmed):
            _manipulator = manipulator()
            if(self.check1.checkState()):
                _manipulator.servo1 = 1
            else:
                _manipulator.servo1 = 0
            if(self.check2.checkState()):
                _manipulator.servo2 = 1
            else:
                _manipulator.servo2 = 0
            if(self.check3.checkState()):
                _manipulator.servo3 = 1
            else:
                _manipulator.servo3 = 0
            if(self.check4.checkState()):
                _manipulator.servo4 = 1
            else:
                _manipulator.servo4 = 0
            if(self.check5.checkState()):
                _manipulator.servo5 = 1
            else:
                _manipulator.servo5 = 0
            if(self.check6.checkState()):
                _manipulator.servo6 = 1
            else:
                _manipulator.servo6 = 0
            if(self.check7.checkState()):
                _manipulator.servo7 = 1
            else:
                _manipulator.servo7 = 0
            self.data['manipulators'] = _manipulator
            self.mani_pub.publish(_manipulator)
        
    def armBtnHandler(self):
        if(self.isArmed):
            self.isArmed = False
            self.armButton.setText("NOT ARMED")
        else:
            self.armButton.setText("ARMED")
            self.isArmed = True
            
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
    
    def thruster_callback(self,thruster):
        self.q_thruster.put(thruster)
    def orientation_callback(self,msg):
        self.q_orientation.put(msg)
        
    def depth_callback(self,depth):
        self.q_depth.put(depth)

    def controller_callback(self,controller):
        self.q_controller.put(controller)
    
    def rel_pos_callback(self,pos):
        self.q_rel_pos.put(pos)
        
    def earth_pos_callback(self,earth):
        self.q_earth_pos.put(earth)
    
    def hull_status_callback(self,hull):
        self.q_hull_status.put(hull)
        
    def controller_feedback_callback(self,feedback):
        self.q_controller_feedback.put(feedback)
        
if __name__ == "__main__":
    rospy.init_node('AUV_gui', anonymous=True)
    app = QApplication(sys.argv)
    form = AUV_gui()
    form.show()
    
    app.exec_()
    
    
    
