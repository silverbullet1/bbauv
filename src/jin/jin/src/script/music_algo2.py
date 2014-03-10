import numpy as np
from numpy import linalg as LA
from numpy import *
from numpy import matrix 
import math
import cmath
#Global

thetaM = [-45, 45, 135, 225]
v = 1500
f = 30000.0
lamda = v / f
d = 0.015
a = math.sqrt(2 * math.pow(d / 2, 2))
A = zeros((4, 360), dtype=complex)
DOA2 = 0

def music_algo2(ls):
    global DOA2, A
    r_conv = 0
    for x in range(int(take)):
        #Array Steering Vector
        pmusic = zeros((90))
        for a_theta in range(0, 90):
            theta = 2*math.pi/lamda*a*math.sin(np.deg2rad(a_theta))
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

    # print max(pmusic)
    #DOA = np.argmax(pmusic)
    with open('pmusic.txt', 'a') as pfile:
        pfile.write("....BEGIN OF PMUSIC FILE....")
        for pm in pmusic:
            pfile.write(str(pm))
            pfile.write("\n")
        pfile.write("....END OF PMUSIC FILE....")
    plt.xlabel("DOA(degree)")
    plt.ylabel("Pmusic")
    plt.title(time.strftime('%x'))
    plt.plot([i for i in range(360)], pmusic) 
    plt.savefig(str(datetime.now())+".png")
    plt.clf()
    count = 0
    for i in pmusic:
        if i == max(pmusic):
            break
        count = count + 1
    DOA = count + 1
