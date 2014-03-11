#!/usr/bin/env python 
import rospy	#This is for logging and printing out error messages to command line
import roslib	#This is to load the correct directory for acoustic to communicate with ROS
import time		#Used for Process sleep

#For plotting the music output
import matplotlib.pyplot as plt
from datetime import datetime

#Deriving the message types for ROS communication
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from nav_msgs.msg import Odometry
from std_msgs.msg._Float32 import Float32

#For state machines for mission planner
import actionlib

#For maths operations
import numpy as np
from numpy import *
from numpy import linalg as LA
from numpy import matrix 
import math
import cmath
#import array

#For TCP communication with sbRIO
import socket 
TCP_IP = '192.168.1.149'
PORT = 5100
BUFFER_SIZE = 350

#Getting keyboard input
import sys 

#Constants
DEGREE_PI = 180
DEGREE_TWO_PI = 360
sampleAmount = sys.argv[1]	#Global constant
isTest = sys.argv[2]
distanceConst = [4,3,2]
counter = 0
fh = 30000.0
fl = 22500.0
DOA = 0
DOA_final = 0


#Updated global variables through subscriptions
heading = 0
depth = 0.0	
elevationAngle3 = 0
#Initialize TCP socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)
(TCP_connect, addr) = s.accept()	#Global socket

'''
	=================== Initialize services ===================
	Services being used are:
		1. locomotionClient	- 	Server to controcomplexList motion of AUV
		2. setServer		- 	ControcomplexList motion of AUV
'''
try:
    locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
    locomotionClient.wait_for_server(timeout=rospy.Duration(5))
except Exception as e: rospy.loginfo("Error connecting to locomotion server")

#Setting controller server	
setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
setServer(forward=True, sidemove=True, heading=True, depth=True, pitch=True, roll=True, topside=False, navigation=False)
'''
	=================== END of Initialize services ===================
'''
'''	
	====================== Saving information from various Subscription ======================
'''
def compass_callback(data):
    global heading
    heading = data.yaw
	
def depth_callback(data): 
    global depth
    depth = data.depth 
'''	
	====================== END of Saving information from various Subscription ======================
'''
'''
	=================== Initialize Subscriptions ===================
	Subscriptions to:
		1. earth_odom	-	Get AUV's current absolute (x,y) coordinate	("earth_odom")
		2. DVL			- 	Get AUV's current relative (x,y) coordinate	("/WH_DVL_data")
		3. depth 		- 	Get AUV's current depth						(/depth)
'''
depth_sub = rospy.Subscriber("/depth", bbauv_msgs.msg.depth, depth_callback)  			#Initialise depth_sub
compass_sub = rospy.Subscriber('/euler', bbauv_msgs.msg.compass_data, compass_callback)	#Initialise compass_sub
'''
	===================END of Initialize Subscriptions ===================
'''
'''
	==================================================================	
	This telcomplexList locomotionClient to move the robot in 
		'f'-Forward distance   (m)		'turn'-bearing of heading (deg)
'''
def sendMovement(forward=0.0, turn=None, depth=1.0):
    global heading
    global locomotionClient
    turn = (turn+heading)%360 if turn else heading
    goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=0.0, depth_setpoint=depth)
    locomotionClient.send_goal(goal)
    if isTest:
        locomotionClient.wait_for_result(rospy.Duration(0.5))
    else:
        locomotionClient.wait_for_result()

def splitTCPMsg(data, num):
    try:
        (real0, imag0,
         real1, imag1,
         real2, imag2,
         real3, imag3,
         real4, imag4,
         real5, imag5,
         real6, imag6,
         real7, imag7) = data.split(',')
        a = [real0, imag0, real1, imag1, real2, imag2, real3, imag3] if num == 1 else [real4, imag4, real5, imag5, real6, imag6, real7, imag7]
    except ValueError:
        print "Error"
        return False
        

    if("NaN" in a or "Inf" in a):
        return False
    else:
        hydro0_complex = np.complex(float(a[0]), float(a[1]))
        hydro1_complex = np.complex(float(a[2]), float(a[3]))
        hydro2_complex = np.complex(float(a[4]), float(a[5]))
        hydro3_complex = np.complex(float(a[6]), float(a[7]))
        return [hydro0_complex,hydro1_complex, hydro2_complex, hydro3_complex]

'''
	=================== Mathematical Methods ===================
'''
def pingerInSight(angle):
    return angle >= 340 and angle <= 20

	
def computeCovarianceMatrix(complexList):
	r_conv = zeros((4, 4), dtype=complex)
	r_conv = np.matrix(r_conv)
	for T in range(int(sampleAmount)-1):
		R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
		cov = R * R.T.conj()
		r_conv += cov
	r_conv = r_conv/(int(sampleAmount)-1)
	return r_conv

def getMax(arrayList):
	max_val = 0
	phiCap = 0
	thetaCap = 0
	for theta in range(len(arrayList[0])):
		for phi in range(len(arrayList)):	
			if max_val < arrayList[phi, theta]:
				max_val = arrayList[phi, theta]
				phiCap = phi
				thetaCap = theta
	return [phiCap,thetaCap]

def classical_3d(complexList):
    phiCap = 0
    thetaCap = 0
    gamma = [-45, 45, 135, 225]
    v = 1500
    f = 30000.0
    lamda = v / f
    d = 0.015
    r = math.sqrt(2 * math.pow(d / 2, 2))
    A = zeros((4, 1), dtype=complex)
    classical = zeros((360, 90))
    R = np.matrix([[(complexList[3])[0]], [(complexList[3])[1]], [(complexList[3])[2]], [(complexList[3])[3]]])
    for theta in range(90):		#Theta is altitude
        for phi in range(360):	#Phi is azimuth
            for i in range(4):	#Hydrophone positions
                pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
                A[i] = cmath.exp((1j)*pd)
        
            Ahat = np.matrix(A)
        S = Ahat*Ahat.T.conj()
        aTSa=(R.T.conj())*S*R
        classical[phi,theta] = aTSa.real
        #plot(classical[:,theta],theta)
        #writeToFile('classical.txt',classical[:,theta],theta)
    [phiCap, thetaCap] = getMax(classical)
    print ("Classical DOA calculated: " + str(phiCap))
    print ("Classical elevation calculated: " + str(thetaCap))	
    return [phiCap,thetaCap]

def music_3d(complexList, f):
	gamma = [-45, 45, 135, 225]
	v = 1500
	lamda = v / f
	d = 0.015
	r = math.sqrt(2 * math.pow(d / 2, 2))
	A = zeros((4, 1), dtype=complex)
	pmusic = zeros((360, 90))
	
	(eigval, eigvec) = LA.eigh(computeCovarianceMatrix(complexList))
	Vn = eigvec[:, 0:3]
	
	for theta in range(90):		#Theta is altitude
		for phi in range(360):	#Phi is azimuth
			for i in range(4):	#Hydrophone positions
				pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
				A[i] = cmath.exp((1j)*pd)
		
			Ahat = np.matrix(A)
			num = Ahat.T.conj() * Ahat
			denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
			pmusic[phi, theta] = num.real / denom.real
			#plot(pmusic[:,theta],theta)
			#writeToFile('pmusic.txt',pmusic[:,theta],theta)
	
	[Music_phiCap,Music_thetaCap] = getMax(pmusic)
	print ("Music DOA calculated: " + str(Music_phiCap))    
	print ("Music elevation calculated: " + str(Music_thetaCap))	
	return [Music_phiCap,Music_thetaCap]

def plot(list,Phi):
    plt.plot([i for i in range(360)], list) 
    plt.savefig(str(datetime.now())+"_"+str(Phi)+".png")	
	
def writeToFile(fileName,list,theta):
    with open(fileName, 'a') as pfile:
        pfile.write("....BEGIN OF " + str(theta) + " FILE....")
        for pm in list:
            pfile.write(str(pm))
            pfile.write("\n") 
        pfile.write("....END OF " + str(theta) + " FILE....")

#Getting data from TCP
def getRawData(conn,num):
    complexList = []
    sleepAwhile(5)		#Wait for certain interval before receive new data
    conn.close()
    (conn, addr) = s.accept()
    while True:
        if len(complexList) == int(sampleAmount):
            break
        else:
            data = conn.recv(BUFFER_SIZE)
            if(splitTCPMsg(data, num)):
                    splitted = splitTCPMsg(data, num)
                    complexList.append(splitted)
                    conn.close()
                    (conn, addr) = s.accept()
                    print("Number of takes: " + str(len(complexList)))
            else:
                print "Proceed to continue"
                conn.close()
                (conn, addr) = s.accept()
                continue

    return [complexList[1],complexList[2], complexList[3]]
				
def sleepAwhile(durationSec=5):
	time.sleep(durationSec)
	
def distanceToPinger(algo_type, num):
    global depth, elevationAngle3, distanceConst, counter, DOA
    pingerDistance = 0
    if algo_type == "MUSIC":
            final_rawData = getRawData(TCP_connect, num)
            if(num == 1):
                [DOA3,elevationAngle3] = music_3d(final_rawData, fh) if len(final_rawData) is not 0 else [0,0]
                DOA = DOA3
            else:
                [DOA3,elevationAngle3] = music_3d(final_rawData, fl) if len(final_rawData) is not 0 else [0,0]
                DOA = DOA3


    elif algo_type == "CLASSICAL" :
            final_rawData = getRawData(TCP_connect)
            [DOA3,elevationAngle3] = classical_3d(final_rawData) if len(final_rawData) is not 0 else [0,0]
    else: 
        print "ERROR: Algorithm was not specified"

    if not overShotPinger(DOA):
        print ("Turning to face pinger at relative " + str(DOA3) + " degrees" )
        sendMovement(turn=DOA)
        pingerDistance = float(distanceConst[counter]) if counter < 3.0 else 1.0
    else:
        pingerDistance = pingerDistance/2.0
    counter+= 1
    print ("AUV is " + str(pingerDistance) + " m away")
    return pingerDistance
	
def overShotPinger(angle):
    return 135<= angle <=225

if __name__ == "__main__":
    #Initialization of Node & Publisher
    rospy.init_node("acoustic_independent") 
	#Printing out of AUV location
    sendMovement(depth=1.0)
    print ("The AUV's bearing is: " + str(heading) + " degrees\n")
    print("Depth: " + str(depth) + " m\n")
	
    while True:
        distanceToMove = distanceToPinger("MUSIC", 1)
        if counter > 1 and overShotPinger(DOA):
            print "Found First, overshot"
            sendMovement(forward=-0.5) 
            print "Reversed"
            distanceToMove = distanceToPinger("MUSIC", 2)
            sendMovement(turn=DOA)
            print("Turning to: " + str(DOA))
            sendMovement(forward=1.0)
            print("Moved forward")
            sendMovement(depth=0)

        elif elevationAngle3 > 30:
            sendMovement(forward=distanceToMove)	#Move toward the pinger
        else:
            print ("Found First, elevation less")
            distanceToMove = distanceToPinger("MUSIC", 2)
            sendMovement(turn=DOA)
            print("Turning to: " + str(DOA))
            sendMovement(forward=1.0)
            print("Moved forward")
            sendMovement(depth=0)
            break
    rospy.spin()
    TCP_connect.close()
