# -*- coding: utf-8 -*-
"""
Created on Wed Mar  13 22:37:12 2020

@author: Zhaoliang
"""

import numpy as np
from scipy import linalg


def solve_discrete_riccati(A, H, Q, R):
    """
    This function is to calculate the discrete steady-state sigma
    using the linalg function from scipy library
    """
    Sigma = linalg.solve_discrete_are(A.T, H.T, Q, R)
    return Sigma

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

def KF_planning(A, B, H, Q, R,dt):
    kfdr = kf_dead_reckoning(A = A, B=B, H =H, Q = Q, R=R)
    if dt == 0:
        sigma_dr = kfdr.initial()
    else:
        for i in range(dt):    
            sigma_dr = kfdr.predict_update()
            #print(sigma_dr)
    return sigma_dr

def KF_planning_Uz(A, B, H, Q, R,dt):
    sigma_dr = KF_planning(A, B, H, Q, R,dt)
    S = np.dot(np.dot(A,sigma_dr),A.T)+Q
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
    
    uts = [0]
    ts = [100]
    tall = np.sum(ts)
    sigma_dr = KF_planning(A, B, H, Q, R,tall)
    S = np.dot(np.dot(A,sigma_dr),A.T)+Q
    U_sigma = np.dot(np.dot(H,S),H.T)+R

    