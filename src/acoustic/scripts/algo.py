import numpy as np
from numpy import linalg as LA
from numpy import *
import math
import cmath
import array

def music_algo(data):

    def splitMsg(data):
        ls = []
        (real0, imag0,
         real1, imag1,
         real2, imag2,
         real3, imag3) = data.split(',')
        hydro0_complex = np.complex(float(real0), float(imag0))
        hydro1_complex = np.complex(float(real1), float(imag1))
        hydro2_complex = np.complex(float(real2), float(imag2))
        hydro3_complex = np.complex(float(real3), float(imag3))
        ls.append(hydro0_complex)
        ls.append(hydro1_complex)
        ls.append(hydro2_complex)
        ls.append(hydro3_complex)
        return ls

    thetaM = [-45, 45, 135, 225]
    v = 1500
    f = 28000.0
    lamda = v / f
    d = 0.015
    a = math.sqrt(2 * math.pow(d / 2, 2))
    A = zeros((4, 360), dtype=complex)
    cmpx_ls = splitMsg(data)
    pmusic = zeros((360))
    for theta in range(1, 361):
        for i in range(0, 4):
            pd = a * math.cos((theta - thetaM[i]) / 180.0 * (math.pi))
            A[i, theta - 1] = cmath.exp(2 * (1j) * math.pi * pd / lamda)
    A = np.matrix(A)
    R = np.matrix([[cmpx_ls[0]], [cmpx_ls[1]],
                   [cmpx_ls[2]], [cmpx_ls[3]]])
    cov = R * R.T.conj()
    (eigval, eigvec) = LA.eigh(cov)
    Vn = eigvec[:, 0:3]
    for phi in range(1, 361):
        Ahat = A[:, phi - 1]
        num = Ahat.T.conj() * Ahat
        denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
        pmusic[phi - 1] = num.real / denom.real

    # print max(pmusic)
    DOA = np.argmax(pmusic)

    return DOA
    '''
    count = 0
    for i in pmusic:
        if i == max(pmusic):
            break
        count = count + 1
    DOA = count + 1
    '''
