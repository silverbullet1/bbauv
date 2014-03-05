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
angFromPing2 = []
TCP_IP = '192.168.1.149'
PORT = 5100

BUFFER_SIZE = 100  # Normally 1024, but we want fast response
data = "NO MESSAGE"
heading = {'yaw':0, 'pitch':0, 'roll':0}
pos = {'x':0, 'y':0}
abs_pos = {'x':0, 'y':0}
depth = 0.0
DOA = 0
#Forward Constant
forward = 2
#Angle Constant
angConst = 10

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)

#Global

take = sys.argv[1]
thetaM = [-45, 45, 135, 225]
v = 1500
f = 30000.0
lamda = v / f
d = 0.015
a = math.sqrt(2 * math.pow(d / 2, 2))
A = zeros((4, 360), dtype=complex)

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
    locomotionClient.wait_for_result()

def getBetterAngle(angle, heading):
    sendMovement(h = angle, d = depth)
#Subscibers
try:
    locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
    locomotionClient.wait_for_server(timeout=rospy.Duration(5))
except Exception as e:
    rospy.loginfo("Error connecting to locomotion server")
#navClient = rospy.ServiceProxy("/navigate2D", navigate2d)
#navClient.wait_for_service()
rospy.ServiceProxy("/set_controller_srv", set_controller)
setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
setServer(forward=True, sidemove=True, heading=True, depth=True, pitch=True, roll=False, topside=False, navigation=False)
#abs_pos_sub = rospy.Subscriber("earth_odom", Odometry, DVLcallback)
#depth_sub = rospy.Subscriber("/depth", bbauv_msgs.msg.depth, depth_callback)
compass_sub = rospy.Subscriber('/euler', bbauv_msgs.msg.compass_data, compass_callback)

#ROS_CALLBACKS

def withinRange(angle):
    return angle >= 0 and angle <= 45

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
    plt.plot([i for i in range(360)], pmusic) 
    plt.savefig(str(datetime.now())+".png")
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

def neotriangulate(doa1, doa2, R12):
    C = doa2 - doa1
    R2 = R12*((math.sin(np.deg2rad(doa1)))/(math.sin(np.deg2rad(C))))
    return [doa2, abs(R2)]

def triangulate(pt1, pt2):
    x1_pos = pt1.pos['x'] 
    y1_pos = pt1.pos['y']
    x2_pos = pt2.pos['x'] 
    y2_pos = pt2.pos['y']
    doa1_1 = pt1.angFromPing
    doa1_2 = pt2.angFromPing
    #doa2_1 = pt2.angFromPing2
    #doa2_2 = pt2.angFromPing2
    #X,Y Corrdinates swapped
    #return a list of xy coordinates

    if (((math.sin(doa1_1/DEGREE_PI*math.pi)) != 0) and ((math.sin(doa1_1/DEGREE_PI*math.pi)) != 0)):	
        m1_1 = (math.sin(doa1_1/DEGREE_PI*math.pi)/(math.cos(doa1_1/DEGREE_PI*math.pi)))
        #m2_1 = (math.sin(heading2_1/DEGREE_PI*math.pi)/math.cos(heading2_1/DEGREE_PI*math.pi)) 
        c1_1 = y1_pos - (m1_1*x1_pos)
        #c2_1 = y1_pos - (m2_1*x1_pos)
        m1_2 = (math.sin(doa1_2/DEGREE_PI*math.pi)/(math.cos(doa1_2/DEGREE_PI*math.pi)))
        #m2_2 = (math.sin(heading2_2/DEGREE_PI*math.pi)/math.cos(heading2_2/DEGREE_PI*math.pi)) 
        c1_2 = y2_pos - (m1_2*x2_pos)
        #c2_2 = y2_pos - (m2_2*x2_pos)
        constant_matrix1    = np.matrix([[c1_1],[c1_2]])
        #constant_matrix2    = np.matrix([[c2_1],[c2_2]])
        coefficient_matrix1 = np.matrix([[1,m1_1],[1,m1_2]])
        #coefficient_matrix2 = np.matrix([[1,m2_1],[1,m2_2]])
        result_matrix1 	   = coefficient_matrix1.I * constant_matrix1
        #result_matrix2 	   = coefficient_matrix2.I * constant_matrix2
        #finalx = (result_matrix1[0,0] + result_matrix2[0,0])/2
        #finaly = (result_matrix1[1,0] + result_matrix2[1,0])/2
        print result_matrix1
        return [result_matrix1[0,0], result_matrix1[1,0]]
    else:
        print "Division by zero has occured"                                  

def listen_ping(ls,conn,data):
    while len(ls) is not int(take):
        print data
        if data:
            ls.append(splitMsg(data))
            conn.close()
            (conn, addr) = s.accept()
            data = conn.recv(BUFFER_SIZE)
        print len(ls)

if __name__ == "__main__":
    rospy.init_node("ac")
    pub = rospy.Publisher("/acoustic/angFromPing", acoustic)    
    print("X_Pos: " + str(abs_pos['x']) + "\t\t" + "Y_Pos: " +  str(abs_pos['y']) + "\n")
    print heading['yaw']
    print("Depth: " + str(depth) + "\n")
    (conn, addr) = s.accept()
    data = conn.recv(BUFFER_SIZE)
    if('NaN' in data or 'Inf' in data):
        print "NaNster is here"
    else:
        print "Correct"
    ls = []
    listen_ping(ls,conn,data)
    music_algo(ls)
    print "Relative is: ", DOA, " degrees"
    angFromPing.append(DOA)
    rospy.loginfo("Got First Point")
    if len(angFromPing) is not 2:
        stopRobot()
        rospy.loginfo("Going straight")
        sendMovement(f = forward, d=depth)
        stopRobot()
        ls = []
        conn.close()
        rospy.loginfo("Connection closed")
        for i in range(1000):
            pass
        (conn, addr) = s.accept()
        data = conn.recv(BUFFER_SIZE)
        listen_ping(ls, conn,data)
        music_algo(ls)
        angFromPing.append(DOA)
    a = neotriangulate(angFromPing[0], angFromPing[1], forward)
    print("Location of Pinger: Distance of " + str(a[1])+"\n")
    print a[0]
    sendMovement(h=(a[0] + heading['yaw'])%360)
    sendMovement(f=a[1])
    rospy.spin()
    conn.close()

