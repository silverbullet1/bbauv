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
import time

DEGREE_PI = 180
DEGREE_TWO_PI = 360
angFromPing = []
TCP_IP = '192.168.1.149'
PORT = 5100
BUFFER_SIZE = 100
heading = {'yaw':0, 'pitch':0, 'roll':0}
pos = {'x':0, 'y':0}
abs_pos = {'x':0, 'y':0}
depth = 0.0
DOA = 0
forward = 2
ls = []

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)
#Global socket
(conn, addr) = s.accept()


#Global constant

take = sys.argv[1]
thetaM = [-45, 45, 135, 225]
v = 1500
f = 30000.0
lamda = v / f
d = 0.015
a = math.sqrt(2 * math.pow(d / 2, 2))
A = zeros((4, 360), dtype=complex)
for theta in range(1, 361):
    for i in range(0, 4):
        pd = a * math.cos((theta - thetaM[i]) / 180.0 * (math.pi))
        A[i, theta - 1] = cmath.exp(2 * (1j) * math.pi * pd / lamda)
A = np.matrix(A)
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

#Subscibers
try:
    locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
    locomotionClient.wait_for_server(timeout=rospy.Duration(5))
except Exception as e:
    rospy.loginfo("Error connecting to locomotion server")
setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
setServer(forward=True, sidemove=True, heading=True, depth=True, pitch=True, roll=False, topside=False, navigation=False)
abs_pos_sub = rospy.Subscriber("earth_odom", Odometry, DVLcallback)
depth_sub = rospy.Subscriber("/depth", bbauv_msgs.msg.depth, depth_callback)
compass_sub = rospy.Subscriber('/euler', bbauv_msgs.msg.compass_data, compass_callback)

#MATH_FUNCTIONS

def withinRange(angle):
    return angle >= 10 and angle <= 30

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

def music_algo(ls, intype):
    r_conv = 0
    for x in range(int(take)):
        pmusic = zeros((360))
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
    if intype == "first":
        for i in pmusic:
            if i == max(pmusic):
                break
            count = count + 1
        DOA = count + 1
    elif intype == "second":
        for i in pmusic:
            if i == max(pmusic[0:90]):
                break
            count = count + 1
        DOA = count + 1

def triangulate(doa1, doa2, R12):
    C = doa2 - doa1
    R2 = R12*((math.sin(np.deg2rad(doa1)))/(math.sin(np.deg2rad(C))))
    return [doa2, abs(R2)]


def listen_ping(ls,conn, intype):
    ls = []
    #Wait for certain interval before receive new data
    time.sleep(5)
    conn.close()
    (conn, addr) = s.accept()
    data = conn.recv(BUFFER_SIZE)
    while len(ls) is not int(take):
        print data
        if data:
            ls.append(splitMsg(data))
            conn.close()
            (conn, addr) = s.accept()
            data = conn.recv(BUFFER_SIZE)
        print("Number of takes: " + str(len(ls)))
    music_algo(ls, intype)

def validateData(data):
    if('NaN' in data or 'Inf' in data):
        return "NaNster is here"
    else:
        return "Correct"

if __name__ == "__main__":
    #Inialisation of Node & Publisher
    rospy.init_node("acoustic_independent")
    pub = rospy.Publisher("/acoustic/angFromPing", acoustic)    
    #Logging of data
    print("X_Pos: " + str(abs_pos['x']) + "\t\t" + "Y_Pos: " +  str(abs_pos['y']) + "\n")
    print heading['yaw']
    print("Depth: " + str(depth) + "\n")
    #Listrning to First Ping
    listen_ping(ls, conn, "first")
    print ("First DOA: " + str(DOA))
    #Ensure pinger always on right of pinger
    if not withinRange(DOA):
        temp = (heading['yaw'] + (DOA - 20))%360
        print("Turning " + str(temp))
        sendMovement(h=(heading['yaw'] + (DOA - 20))%360)
        listen_ping(ls, conn, "first")
    angFromPing.append(DOA)
    print("After minus 20 is " + str(DOA))
    rospy.loginfo("Obtained first DOA")
    sendMovement(f=forward)
    time.sleep(5)
    listen_ping(ls, conn, "second")
    angFromPing.append(DOA)
    print DOA
    result = triangulate(angFromPing[0], angFromPing[1], forward)
    print result[1], result[0]
    sendMovement(f=0.4)
    sendMovement(h=(result[0] + heading['yaw'])%360)
    sendMovement(f=result[1])
    rospy.spin()
    conn.close()


