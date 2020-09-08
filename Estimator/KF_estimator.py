# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:20:15 2020

@author: Zhaoliang
"""

import numpy as np
import Simulation.Generate_seq_u as Gsequ
import Simulation.Simulator as Simu
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata


class KalmanFilter(object):
    def __init__(self, A = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(A is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = A.shape[1]
        self.m = H.shape[1]
        self.A = A
        self.H = H
        self.B = np.zeros((self.n, 1)) if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.m) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K,self.H),self.P)
        return self.P,self.x
      
def KF_estimator(A,B,H,Q,R,measurements):
    
    kf = KalmanFilter(A=A,B=B,H=H,Q=Q,R=R)
    
    predictions = []
    estimates = []
    Sigma = []
    for z in measurements:
        predictions.append(kf.predict())
        P,x=kf.update(z)
        estimates.append(x)
        Sigma.append(P)

    return Sigma,estimates

if __name__ == '__main__':
    dx = 2
    dz = 2
    A = np.array([[1,0],[0,1]]).reshape(dx, dx)
    B = np.array([[1],[1]]).reshape(dx, 1) 
    H = np.array([[1,0],[0,1]]).reshape(dz, dx)
    Q = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(dx, dx)
    R = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(dz, dz)
    x0 = np.array([[0],[0]]).reshape(dx, 1)
    uts = [0,1,0,2]
    ts = [100,10,20,10]
    u = Gsequ.generate_sequential_ut(uts,ts)
    y,z = Simu.Simulator(A,B,H,Q,R,x0,u)
    Sigma,estimates = KF_estimator(A,B,H,Q,R,z)
    
    # Plot figure
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)