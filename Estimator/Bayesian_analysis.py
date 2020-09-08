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

def Bayesian_analysis(A, B, H, Q, R,Sigma,estimates,measurements):
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
    u = Gsequ.generate_sequential_ut(uts,ts)
    y,z = Simu.Simulator(A,B,H,Q,R,x0,u)
    Sigma,estimates = KF_est.KF_estimator(A,B,H,Q,R,z)
    
    """
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    """
    ut_zt,Sigma_ut_zt = Bayesian_analysis(A, B, H, Q, R,Sigma,estimates,z)
    