# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 21:30:28 2020

@author: Zhaoliang
"""

import numpy as np
from scipy import linalg

class kf_dead_reckoning(object):
    def __init__(self, A = None, B = None, H = None, Q = None, R = None, P = None):

        if(A is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = A.shape[1]
        self.m = H.shape[1]
        self.A = A
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.dot(np.eye(self.n),10) if P is None else P
    
    def initial(self):
        return self.P
    def predict_update(self):
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.P = self.P - np.dot(np.dot(K,self.H),self.P)
        return self.P    

def KF_planner(SM,dt):
    """
    A,B,H,Q,R are the system matrices
    dt is the timestep, dt is a scaler
    """
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    kfdr = kf_dead_reckoning(A = A, B=B, H =H, Q = Q, R=R)
    if dt == 0:
        Sigma_xhat = kfdr.initial()
    else:
        for i in range(dt):    
            Sigma_xhat = kfdr.predict_update()
            #print(sigma_dr)
    return Sigma_xhat

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
    Sigma_xhat = KF_planner(A, B, H, Q, R,T)