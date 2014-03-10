#!/usr/bin/env python 
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import rospy
import roslib
from numpy import linalg as LA
from nav_msgs.msg import Odometry
from numpy import *
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from numpy import matrix 
import math
import cmath
import array
import socket 
import sys 
from std_msgs.msg._Float32 import Float32
import actionlib

DEGREE_PI = 180
DEGREE_TWO_PI = 360
angFromPing = []
TCP_IP = '192.168.1.149'
PORT = 5100

BUFFER_SIZE = 100  # Normally 1024, but we want fast response
data = "NO MESSAGE"
heading = {'yaw':0, 'pitch':0, 'roll':0}
pos = {'x':0, 'y':0}
abs_pos = {'x':0, 'y':0}
depth = 0.0

DOA = 0

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)

#Global

take = sys.argv[1]
thetaM = [-45, 45, 135, 225]
v = 1500
f = 28000.0
lamda = v / f
d = 0.015
a = math.sqrt(2 * math.pow(d / 2, 2))
A = zeros((4, 360), dtype=complex)

class Point():
    heading = 0
    pos = {'x': 0, 'y':0}
    angFromPing = 0

    def __init__(self, x, y, heading, angle):
        self.heading = heading
        self.pos['x'] = x
        self.pos['y'] = y
        self.angFromPing = angle

#ROS_CALLBACKS

def stopRobot():
    sendMovement(f=0, sm=0)

def DVLcallback(data):
    global abs_pos
    abs_pos['x'] = data.pose.pose.position.x
    abs_pos['y'] = data.pose.pose.position.y

def compass_callback(data):
    global heading
    heading['yaw'] = data.yaw

def depth_callback(data):
    global depth
    depth = data.depth 


def sendMovement(f=0.0, h=None, sm=0.0, d=None):
    d = d if d else depth
    h = h if h else heading['yaw']
    global locomotionClient
    goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h, sidemove_setpoint=sm, depth_setpoint=d)
    locomotionClient.send_goal(goal)
    locomotionClient.wait_for_result(rospy.Duration(0.5))

def getBetterAngle(angle, heading):
    sendMovement(h = angle, d = depth)
#Subscibers
try:
    locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
    locomotionClient.wait_for_server(timeout=rospy.Duration(5))
except Exception as e:
    rospy.loginfo("Error connecting to locomotion server")
controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
controllerSettings.wait_for_service()
navClient = rospy.ServiceProxy("/navigate2D", navigate2d)
navClient.wait_for_service()
rospy.ServiceProxy("/set_controller_srv", set_controller)
setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
setServer(forward=True, sidemove=True, heading=True, depth=False, pitch=True, roll=False, topside=False, navigation=False)

abs_pos_sub = rospy.Subscriber("earth_odom", Odometry, DVLcallback)
depth_sub = rospy.Subscriber("/depth", bbauv_msgs.msg.depth, depth_callback)
compass_sub = rospy.Subscriber('/euler', bbauv_msgs.msg.compass_data, compass_callback)

#ROS_CALLBACKS

def withinRange(angle):
    return 0 <= angle <= 360

def splitMsg(data):

    (real0, imag0,
     real1, imag1,
     real2, imag2,
     real3, imag3) = data.split(',')

    hydro0_complex = np.complex(float(real0), float(imag0))
    hydro1_complex = np.complex(float(real1), float(imag1))
    hydro2_complex = np.complex(float(real2), float(imag2))
    hydro3_complex = np.complex(float(real3), float(imag3))
    
    return [hydro0_complex,hydro1_complex, hydro2_complex, hydro3_complex]

def music_algo(ls):
    global DOA, A
    r_conv = 0
    for x in range(int(take)):
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
    r_conv = r_conv/int(take)
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
        pfile.write("....BEGIN OF PMUSIC FILE....")
        for pm in pmusic:
            pfile.write(str(pm))
            pfile.write("\n")
        pfile.write("....END OF PMUSIC FILE....")

    count = 0
    for i in pmusic:
        if i == max(pmusic):
            break
        count = count + 1
    DOA = count + 1

def triangulate(pt1, pt2):

    h1 = pt1.heading
    h2 = pt2.heading
    x1_pos = pt1.pos['x'] 
    y1_pos = pt1.pos['y']
    x2_pos = pt2.pos['x'] 
    y2_pos = pt2.pos['y']
    doa1 = pt1.angFromPing
    doa2 = pt2.angFromPing
    #X,Y Corrdinates swapped
    #return a list of xy coordinates
    heading1 = (DEGREE_TWO_PI+h1+doa1) % (DEGREE_TWO_PI)
    heading2 = (DEGREE_TWO_PI+h2+doa2) % (DEGREE_TWO_PI)
    if (((math.sin(heading1/DEGREE_PI*math.pi)) != 0) and ((math.sin(heading2/DEGREE_PI*math.pi)) != 0)):	
        m1 = (math.sin(heading1/DEGREE_PI*math.pi))	/ (math.cos(heading1/DEGREE_PI*math.pi)) 
        c1 = y1_pos - (m1*x1_pos)
        m2 =   (math.sin(heading2/DEGREE_PI*math.pi)) / (math.cos(heading2/DEGREE_PI*math.pi))
        c2 = y2_pos - (m2*x2_pos)
        constant_matrix    = np.matrix([[c1],[c2]])
        coefficient_matrix = np.matrix([[1,m1],[1,m2]])
        result_matrix 	   = coefficient_matrix.I * constant_matrix
        return [result_matrix[0,0],result_matrix[1,0]]                                      
    else:
        print "Division by zero has occured"                                  
if __name__ == "__main__":
    rospy.init_node("ac")
    pub = rospy.Publisher("/acoustic/angFromPing", bbauv_msgs.msg.acoustic)
    while len(angFromPing) is not 2:
        print(str(abs_pos['x']) + "\t\t" + str(abs_pos['y']) + "\n")
        print(str(heading['yaw']))
        (conn, addr) = s.accept()
        data = conn.recv(BUFFER_SIZE)
        if('NaN' in data or 'Inf' in data):
            print "NaNster is here"
        else:
            print "Correct"
        ls = []
        while len(ls) is not int(take):
            stopRobot()
            if data:
                ls.append(splitMsg(data))
                conn.close()
                (conn, addr) = s.accept()
                data = conn.recv(BUFFER_SIZE)
            print data
        music_algo(ls)
        if withinRange(DOA):
            angFromPing.append(Point(abs_pos['x'], abs_pos['y'], heading['yaw'], DOA))
        else:
            getBetterAngle(DOA, heading['yaw'])
        sendMovement(sm = 5, d=depth)
        ls = []
        print "DOA is: ", DOA, " degrees"
    a = triangulate(angFromPing[0], angFromPing[1])
    print("Location of Pinger: " + str(a[0]) + "\t:" + str(a[1]))
    navClient(a[0], a[1])
    rospy.spin()
    conn.close()

