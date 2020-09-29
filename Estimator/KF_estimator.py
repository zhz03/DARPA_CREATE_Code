# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:20:15 2020

@author: Zhaoliang
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
        predictions.append(kf.predict())
        P,x=kf.update(z)
        estimates.append(x)
        Sigma.append(P)

    return Sigma,estimates#,predictions

def figs_verification(num):
    dx = 1
    Arange = [1,1]
    Brange = [1,1]
    Hrange = [0,2]
    Qrange = [0,2]    
    Rrange = [0,2]
    System_models = SMGen1d.SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange)
    #As,Hs,Bs,Qs,Rs,
    T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
    SM_num = len(System_models[0])
    T = 1000
    ts[0] = T
    for i in range(SM_num):
        A = System_models[0][i]
        B = System_models[1][i]
        H = System_models[2][i]
        Q = System_models[3][i]
        R = System_models[4][i]
        SM = [A,B,H,Q,R]
        u = Gsequ.generate_sequential_ut(uts,ts)
        y,z = Simu.Simulator(SM,x0,u)
        Sigma,estimates = KF_estimator(SM,z)
        
        ground_truth = cnvdata.convert_array2list_nd(y,dx)
        measurements = cnvdata.convert_array2list_nd(z,dx) 
        estimates = cnvdata.convert_array2list_nd(estimates,dx)
        errors = [estimates[0][i] - ground_truth[0][i] for i in range(len(estimates[0]))]
        #predictions = cnvdata.convert_array2list_nd(predictions,dx)
        plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name = './figs/KF_estimator_figs/' + str(i) + '.jpg'
        plt.savefig(fig_name)
        plt.close()

        CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = 0, Sigma_pln = Sigma[-1][0][0])
        CompP2S.visualization_compare(errors,1)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name1 = './figs/KF_estimator_figs/' + str(i) + '_error.jpg'
        plt.savefig(fig_name1)
        plt.close()
        
if __name__ == '__main__':
    figs_verification(20)

    """
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
    SM = [A,B,H,Q,R]
    """
    """
    dx = 1
    dz = 1
    # system matrices parameters
    q = 0.3
    r = 0.2
    a = 1
    h = .1
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    uts = [0,1]
    ts = [100,1]
    ut = [0,1] 
    trials = 1000
    SM = [A,B,H,Q,R]
    """
    """
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
    S = np.dot(H,np.dot(A,estimates[-2]))
    ut_zt1 = z[-1] - S
    
    ut_zt,Sigma_ut_zt = BA.Bayesian_analysis(SM,Sigma,estimates,z)
    ut = [np.dot(H,B)*ut[0],np.dot(H,B)*ut[1]]
    u_D = DM.Decision_making(ut,ut_zt,Sigma_ut_zt)
    #ut_zt1 = z[-1] - np.dot(H,predictions[-2])
    # Plot figure
    """
    """
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    #predictions = cnvdata.convert_array2list_nd(predictions,dx)
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
    """
 
        
        