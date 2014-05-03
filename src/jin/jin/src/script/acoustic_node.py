#Beware of radian and degree error 
#Make sure normalise bearing 
#Ensure sub stays stationary while taking angle
#Single Value Decomposition for passive reading

#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('acoustic')
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import numpy as np
from numpy import matrix 
import actionlib
import math
import socket
import signal
from numpy import linalg as LA
from numpy import *
import math
import cmath
import array
import matplotlib.pyplot as plt
from datetime import datetime
import sys

#Constants
DEGREE_TWO_PI = 360
DEGREE_PI = 180
BUFFER_SIZE = 70
TCP_IP = '192.168.1.149'
TCP_PORT = 5100

class Point():
    heading = 0
    pos = {'x': 0, 'y':0}
    angFromPing = 0

    def __init__(self, x, y, heading, angle, angle2):
        self.heading = heading
        self.pos['x'] = x
        self.pos['y'] = y
        self.angFromPing = angle
        self.angFromPing2 = angle2


class AcousticNode(object):

    def __init__(self, param = {'test':False,'boundary':30,'sidemove':1,'takes':5,'active':False}):

        #ROS Params

        self.test = param['test']
        self.boundary = param['boundary']
        self.sidemove = param['sidemove']
        self.takes = param['takes']
        self.active = param['active']
        signal.signal(signal.SIGINT, self.quitProgram)
        self.data = []
        self.depth = 0
        self.heading = {'yaw':0, 'pitch':0, 'roll':0}
        self.pos = {'x':0, 'y':0}
        self.abs_pos = {'x':0, 'y':0}
        self.DOA = 0
        self.points = []
        self.finalDestination = []


        #Signals for state transition

        self.aborted = False
        self.killed = False
        self.completed = False

        #Setup socket connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((TCP_IP, TCP_PORT))
        self.sock.listen()
        (self.conn,self.addr) = self.sock.accept()



    #Python Methods
            
    def grouper(n, iterable, fillvalue=None):
        "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
        args = [iter(iterable)] * n
        return izip_longest(fillvalue=fillvalue, *args)

    #ROS Callbacks

    def handleSrv(self, req):
        if req.start_request:
            self.aborted = False
            self.depth_setpoint = req.start_ctrl.depth_setpoint
            self.active = True
        elif req.abort_request:
            self.aborted = True
        return mission_to_visionResponse(True, False)

    def compass_callback(self,data):
        self.heading['yaw'] = data.yaw

    def depth_callback(self,data):
        self.depth = data.depth 

    def pos_callback(self,data):
        self.pos['x'] = data.pose.pose.position.x
        self.pos['y'] = data.pose.pose.position.y

    def DVL_callback(self, data):
        self.abs_pos['x'] = data.pose.pose.x
        self.abs_pos['y'] = data.pose.pose.y
    #ROS Utility functions

    def sendMovement(self, f=0.0, h=None, sm=0.0, d=None):
        goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h, sidemove_setpoint=sm, depth_setpoint=d)
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(0.5))

    def stopRobot(self):
        self.sendMovement(f=0, sm=0)

    def startSrv_Sub(self):
    #Initialise Locomotion Client 
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except Exception as e:
            rospy.logerr("Error running Locmotion Client")
    #Initialise Controller Service:
        try:
            self.controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
            self.controllerSettings.wait_for_service()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to Controller")
    #Initialise Navigation Service
        try:
            self.navClient = rospy.ServiceProxy("/navigation2D", navigation2d)
            self.navClient.wait_for_service()
        except rospy.ServiceException:
            rospy.logerr("Error connecting to navigate2D server")

    #Initialise connection to mission planner
        self.comServer = rospy.Service("/acoustic/mission_to_vision", mission_to_vision, self.handleSrv)

    #Initialise acoustic to mission 
        try:
            self.toMission = rospy.ServiceProxy("/acoustic/vision_to_mission", vision_to_mission)
            self.toMission.wait_for_service(timeout = 5)
        except rospy.ServiceException:
            rospy.logerr("Error connecting to mission planner")
    #Setting controller server
        self.setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        self.setServer(forward=True, sidemove=True, heading=True, depth=False, pitch=True, roll=False,
                  topside=False, navigation=False)

    #Initialise publisher
        self.angle_pub = rospy.Publisher('/acoustic/angleFromPing', acoustic)

    #Initialise pos_sub
        self.position_sub = rospy.Subscriber("/WH_DVL_data", Odometry ,self.pos_callback)

    #Initialise earth_odom
        self.abs_pos_sub = rospy.Subscriber("earth_odom", Odometry, self.DVL_callback)

    #Initialise depth_sub
        self.depth_sub = rospy.Subscriber("/depth", depth, self.depth_callback)

    #Inialise compass_sub 
        self.compass_sub = rospy.Subscriber('/euler', compass_data, self.compass_callback)

        
    def unregister(self):
        self.position_sub.unregister()
        self.depth_sub.unregister()
        self.compass_sub.unregister()

    def quitProgram(self, signal, frame):
        self.killed = True
        self.aborted = True

    def abortMission(self):
        if not self.test or not self.active:
            self.toMission(fail_request=True, task_complete_request=False)
        self.unregister()
        self.stopRobot()
        self.aborted = True
        self.killed = True

    def taskComplete(self):
        if not self.test and self.active:
            self.toMission(task_complete_request=True)
        self.stopRobot()
        self.aborted = True
        self.killed = True
        self.completed = True

    #Mathematical Methods 

    def normaliseHeading(self, angle, heading):
        return (DEGREE_TWO_PI+heading+angle) % (DEGREE_TWO_PI)

    def withinRange(self, angle):
        return 0 <= angle <= 360 

    def getBetterAngle(self, angle, heading):
            #Turn close to pinger
            self.sendMovement(h = angle)

def triangulate(pt1, pt2):

    h1 = pt1.heading
    h2 = pt2.heading
    x1_pos = pt1.pos['x'] 
    y1_pos = pt1.pos['y']
    x2_pos = pt2.pos['x'] 
    y2_pos = pt2.pos['y']
    doa1_1 = pt1.angFromPing
    doa1_2 = pt2.angFromPing
    doa2_1 = pt2.angFromPing2
    doa2_2 = pt2.angFromPing2
    #X,Y Corrdinates swapped
    #return a list of xy coordinates
    heading1_1 = (DEGREE_TWO_PI+h1+doa1_1) % (DEGREE_TWO_PI)
    heading1_2 = (DEGREE_TWO_PI+h2+doa1_2) % (DEGREE_TWO_PI)
    heading2_1 = (DEGREE_TWO_PI+h1+doa2_1) % (DEGREE_TWO_PI)
    heading2_2 = (DEGREE_TWO_PI+h1+doa2_2) % (DEGREE_TWO_PI) 

    if (((math.sin(heading1_1/DEGREE_PI*math.pi)) != 0) and ((math.sin(heading1_2/DEGREE_PI*math.pi)) != 0) 
            and math.sin(heading2_1/DEGREE_PI*math.pi) and math.sin(heading2_2/DEGREE_PI*math.pi)):	
        m1_1 = math.tan((heading1_1/DEGREE_PI*math.pi),(heading1_1/DEGREE_PI*math.pi)) 
        m2_1 = math.tan((heading2_1/DEGREE_PI*math.pi),(heading2_1/DEGREE_PI*math.pi)) 
        c1 = y1_pos - (m1*x1_pos)
        m2_1 = math.tan((heading1_2/DEGREE_PI*math.pi),(heading1_2/DEGREE_PI*math.pi)) 
        m2_2 = math.tan((heading2_2/DEGREE_PI*math.pi),(heading2_2/DEGREE_PI*math.pi)) 
        c2 = y2_pos - (m2*x2_pos)
        constant_matrix    = np.matrix([[c1],[c2]])
        coefficient_matrix1 = np.matrix([[1,m1_1],[1,m1_2]])
        coefficient_matrix2 = np.matrix([[1,m2_1],[1,m2_2]])
        result_matrix1 	   = coefficient_matrix1.I * constant_matrix
        result_matrix2 	   = coefficient_matrix2.I * constant_matrix
        finalx = (result_matrix1[0,0] + result_matrix2[0,0])/2
        finaly = (result_matrix1[1,0] + result_matrix2[1,0])/2
        return [finalx, finaly]
    else:
        print "Division by zero has occured"                                  
    def splitMsg(self, data):

        (real0, imag0,
         real1, imag1,
         real2, imag2,
         real3, imag3) = data.split(',')

        hydro0_complex = np.complex(float(real0), float(imag0))
        hydro1_complex = np.complex(float(real1), float(imag1))
        hydro2_complex = np.complex(float(real2), float(imag2))
        hydro3_complex = np.complex(float(real3), float(imag3))
        
        return [hydro0_complex,hydro1_complex, hydro2_complex, hydro3_complex]


    def music_algo(self, ls):
        r_conv = 0
        thetaM = [-45, 45, 135, 225]
        v = 1500
        f = 28000.0
        lamda = v / f
        d = 0.015
        a = math.sqrt(2 * math.pow(d / 2, 2))
        A = zeros((4, 360), dtype=complex)

        for x in range(int(self.takes)):
            pmusic = zeros((360))
            for theta in range(1, 361):
                for i in range(0, 4):
                    pd = a * math.cos((theta - thetaM[i]) / 180.0 * (math.pi))
                    A[i, theta - 1] = cmath.exp(2 * (1j) * math.pi * pd / lamda)
            A = np.matrix(A)
            #print ls[x]
            R = np.matrix([[(ls[x])[0]], [(ls[x])[1]],
                           [(ls[x])[2]], [(ls[x])[3]]])
            cov = R * R.T.conj()
            #print cov
            r_conv += cov
        r_conv = r_conv/int(self.takes)
        #print r_conv
        (eigval, eigvec) = LA.eigh(r_conv)
        Vn = eigvec[:, 0:3]
        for phi in range(1, 361):
            Ahat = A[:, phi - 1]
            num = Ahat.T.conj() * Ahat
            denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
            pmusic[phi - 1] = num.real / denom.real
        plt.plot(pmusic, [i for i in range(360)]) 
        plt.savefig(str(datetime.now()))
        # print max(pmusic)
        #DOA = np.argmax(pmusic)
        with open('pmusic.txt', 'a') as pfile:
            pfile.write("....BEGIN....\n")
            for pm in pmusic:
                pfile.write(str(pm))
                pfile.write("\n")
            pfile.write("....END....\n")

        count = 0
        for i in pmusic:
            if i == max(pmusic):
                break
            count = count + 1
        self.DOA = count + 1

    def active_reading(self):
        while len(self.points) is not 2:
            #Output of vehicle's info
            print(str(self.abs_pos['x']) + "\t\t" + str(abs_pos['y']) + "\n")
            print(str(self.heading['yaw']))
            data = self.conn.recv(BUFFER_SIZE)
            if('NaN' in data or 'Inf' in data):
                print "NaNster is here"
            else:
                print "Correct"
            ls = []
            while len(ls) is not int(self.takes):
                self.stopRobot()
                rospy.sleep(2.0)
                if data:
                    ls.append(self.splitMsg(data))
                    self.conn.close()
                    (self.conn, self.addr) = self.sock.accept()
                    data = self.conn.recv(BUFFER_SIZE)
                print data
            self.music_algo(ls)
            if self.withinRange(self.DOA):
                self.points.append(Point(self.abs_pos['x'], self.abs_pos['y'], self.heading['yaw'], self.DOA))
            else:
                self.getBetterAngle(self.DOA, self.heading['yaw'])
            self.sendMovement(sm = 3, d=depth)
            ls = []
            print "DOA is: ", self.DOA, " degrees"
        a = self.triangulate(self.points[0], self.points[1])
        print("Location of Pinger: " + str(a[0]) + "\t:" + str(a[1]))
        self.navClient(a[0], a[1])
        self.conn.close()


if __name__ == "__main__":
    rospy.init_node("acoustic_core")
    acousticCore =  AcousticNode()
    rospy.spin()




