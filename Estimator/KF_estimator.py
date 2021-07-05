# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:20:15 2020

@author: Zhaoliang and Zida 
"""

import numpy as np
import Simulations.Generate_seq_u as Gsequ
import Simulations.Simulator as Simu
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import utility_functions.CompP2SHist as CompP2SHist
import Estimator.Bayesian_analysis as BA
import Estimator.Decision_making as DM
import E2Etest.SM_generator_1d as SMGen1d
import E2Etest.System_setup_generator as SSGen
import matplotlib.pyplot as plt

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
        self.x_last = np.zeros((self.n, 1)) if self.x is None else self.x
        self.p0 = 0.5
        self.p1 = 0.5
        self.p = []

    def predict(self, u = 0):
        self.x_last = self.x
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
    
    def update_MM(self, z, ut):
        mean_ut_zt = []
        Sigma_ut_zt = []    
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
                
        ut_zt = y
        sigma = S

        for i in range(len(ut)):
            mean_ut_zt.append(np.dot(np.dot(self.H,self.B),ut[i]) )
            Sigma_ut_zt.append(sigma)
        # print("ut_zt:{}, Sigma_ut_zt: {}, mean: {}".format(ut_zt, Sigma_ut_zt,mean_ut_zt))
        pdf_H_list, decide_u = DM.Decision_making_MM(ut,ut_zt,mean_ut_zt,Sigma_ut_zt)
             
        if len(ut) == 2: 

            p0 = (pdf_H_list[0]*1000)/((sum(pdf_H_list))*1000)
            p1 = (pdf_H_list[1]*1000)/((sum(pdf_H_list))*1000)

            self.p0 = (self.p0*p0)/(self.p0*p0 + self.p1*p1)
            self.p1 = (self.p1*p1)/(self.p0*p0 + self.p1*p1)
            if self.p1 > 0.99:
                self.p1 = 1
                self.p0 = 0
            elif self.p0 > 0.99:
                self.p0 = 1
                self.p1 = 0
                
            bias = (self.p0) * mean_ut_zt[0] + (self.p1) * mean_ut_zt[1]
                    
            self.x = self.x + np.dot(K, y) 
            self.x_last = self.x
            self.P = self.P - np.dot(np.dot(K,self.H),self.P)
            x_res = self.x - self.x_last
            
            delta_P = np.dot(x_res,x_res.T)
            
            if np.linalg.det(delta_P) < np.linalg.det(self.P):
                self.P = self.P + delta_P
                self.x = self.x + np.dot(1-K, bias)
        else:
            #TODO: len(ut) > 2
            bias = 0 
            
            self.x = self.x + np.dot(K, y)
            self.P = self.P - np.dot(np.dot(K,self.H),self.P)
        
        return self.P,self.x
    
    def update_rkf(self, z):
        mean_ut_zt = []
        Sigma_ut_zt = []    
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        HB = np.dot(self.H, self.B)
        
        M_k = np.dot(np.dot(np.linalg.inv(np.dot(np.dot(HB.T, np.linalg.inv(S)), HB)), HB.T), np.linalg.inv(S))     
        L_k = K + np.dot((np.identity(K.shape[0]) - np.dot(K, self.H)), np.dot(self.B, M_k))
        u_est_k = np.dot(M_k, y)
        
        A_star = (np.identity(self.B.shape[0]) - np.dot(np.dot(self.B, M_k), self.H)) #A* = (I-GMC)
        GM = np.dot(self.B, M_k)
        Q_star = np.dot(np.dot(GM, self.R), GM.T)
        P_star = np.dot(np.dot(A_star, self.P), A_star.T) + Q_star # P* = (I-GMC)P_k|k-1(I-GMC).T + Q*
        S_star = -np.dot(np.dot(self.B, M_k), self.R)
        
        self.x = self.x + np.dot(L_k, y)
        self.P = P_star - np.dot(K, (np.dot(P_star, self.H.T) + S_star).T) 

        return self.P,self.x, u_est_k
      
def KF_estimator(SM,measurements):
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    
    kf = KalmanFilter(A=A,B=B,H=H,Q=Q,R=R)
    
    predictions = []
    estimates = []
    Sigma = []
    for z in measurements:
        predictions.append(kf.predict()) #default u = 0 
        P,x=kf.update(z)
        estimates.append(x)
        Sigma.append(P)

    return Sigma,estimates#,predictions

def KF_estimator_MM(SM,measurements,ut): #MM means multiple model meothod to calibrate x using U models
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    kf = KalmanFilter(A=A,B=B,H=H,Q=Q,R=R)
    
    predictions = []
    estimates = []
    Sigma = []
    
    for z in measurements:
        predictions.append(kf.predict()) #default u = 0 
        P,x=kf.update_MM(z,ut)
        estimates.append(x)
        Sigma.append(P)

    return Sigma,estimates

def KF_estimator_ugt(SM,measurements,u):
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    
    kf = KalmanFilter(A=A,B=B,H=H,Q=Q,R=R)
    
    predictions = []
    estimates = []
    Sigma = []
    for i, z in enumerate(measurements):
        predictions.append(kf.predict(u[i])) 
        P,x=kf.update(z)
        estimates.append(x)
        Sigma.append(P)

    return Sigma,estimates

def KF_estimator_rkf(SM,measurements,u):
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    
    kf = KalmanFilter(A=A,B=B,H=H,Q=Q,R=R)
    
    predictions = []
    x_estimates = []
    u_estimates = []
    Sigma = []
    for _, z in enumerate(measurements):
        predictions.append(kf.predict()) 
        P,x,u_est_k=kf.update_rkf(z)
        x_estimates.append(x)
        Sigma.append(P)
        u_estimates.append(u_est_k)
    return Sigma,x_estimates, u_estimates

if __name__ == '__main__':
    
    dx = 1
    dz = 1
    q = .1254
    r = 1.6798
    r = .1254
    a = 1.4816

    #a = 1.0
    h = 1.5927
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q]).reshape(dx, dx)
    R = np.array([r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    
    uts = [0,1]
    ts = [100,1]
    ut = [0,1] 
    trials = 1000
    SM = [A,B,H,Q,R]
    
    u = Gsequ.generate_sequential_ut(uts,ts)
    y,z = Simu.Simulator(SM,x0,u)
    Sigma,estimates = KF_estimator(SM,z)
    """
    S = np.dot(H,np.dot(A,estimates[-2]))
    ut_zt1 = z[-1] - S
    
    ut_zt,Sigma_ut_zt = BA.Bayesian_analysis(SM,Sigma,estimates,z)
    ut = [np.dot(H,B)*ut[0],np.dot(H,B)*ut[1]]
    u_D = DM.Decision_making(ut,ut_zt,Sigma_ut_zt)
    #ut_zt1 = z[-1] - np.dot(H,predictions[-2])
    # Plot figure
    """

    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    #predictions = cnvdata.convert_array2list_nd(predictions,dx)
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)

 
        
        