# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 21:50:12 2020

@author: Zhaoliang
"""

import numpy as np
import Sensor_Planner.KF_planner_nd as kfplnnd

def Ut_planner_nd(SM,Sigma_xhat):
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    S = np.dot(np.dot(A,Sigma_xhat),A.T)+Q
    U_sigma = np.dot(np.dot(H,S),H.T)+R
    return U_sigma

if __name__ == '__main__':   
    dx = 2
    dz = 2
    q = np.array([[0.2,0.4],[0.4,0.2]]).reshape(dx, dx)
    r = np.array([[0.2,0],[0,0.2]]).reshape(dz, dz)
    
    A = np.array([[1,0],[0,1]]).reshape(dx, dx)
    B = np.array([[1],[1]]).reshape(dx, 1) 
    H = np.array([[1,0],[0,1]]).reshape(dz, dx)
    Q = np.dot(q,q.T)
    R = np.dot(r,r.T)
    x0 = np.array([[0],[0]]).reshape(dx, 1)

    T = 100  
    Sigma_xhat = kfplnnd.KF_planner(A, B, H, Q, R,T)
    Sigma_ut= Ut_planner_nd(A, B, H, Q, R,Sigma_xhat)