# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 23:37:49 2020

@author: Zhaoliang
"""

import numpy as np
import Estimator.KF_estimator as KF_est
import Simulation.Generate_seq_u as Gsequ
import Simulation.Simulator as Simu
import utility_functions.convert_data as cnvdata
import utility_functions.CompP2SHist as CompP2SHist
import matplotlib.pyplot as plt

def Bayesian_analysis(SM,Sigma,estimates,measurements):
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    ut_zt = measurements[-1] - np.dot(H,np.dot(A,estimates[-2]))
    Sigma_t = Sigma[-1]
    S = np.dot(np.dot(A,Sigma_t),A.T)+Q
    Sigma_ut_zt = np.dot(np.dot(H,S),H.T)+R
    return ut_zt,Sigma_ut_zt

if __name__ == '__main__':
    dx = 1
    dz = 1
    # system matrices parameters
    q = 0.2
    r = 0.1
    a = 1
    h = 1
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    uts = [0,1]
    ts = [100,10]
    samp_num = 1000
    Ut_zt = []
    for i in range(samp_num):
        u = Gsequ.generate_sequential_ut(uts,ts)
        SM = [A,B,H,Q,R]
        y,z = Simu.Simulator(SM,x0,u)
        Sigma,estimates = KF_est.KF_estimator(SM,z)
        ut_zt,Sigma_ut_zt = Bayesian_analysis(SM,Sigma,estimates,z)
        Ut_zt.append(ut_zt)
    Ut_zt = cnvdata.convert_array2list_nd(Ut_zt,dx)
    mean_pln = np.dot(H,B)*uts[1]
    CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = np.mean(Ut_zt), Sigma_pln = Sigma_ut_zt[0][0])
    CompP2S.visualization_compare(Ut_zt[0],10)
    plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
    
    """
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    """
    
    