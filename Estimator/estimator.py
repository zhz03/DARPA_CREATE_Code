# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:02:38 2020

@author: Zhaoliang
"""

import numpy as np
import Estimator.KF_estimator as KF_est
import Estimator.Bayesian_analysis as BA
import Estimator.Decision_making as DM
import Simulation.Generate_seq_u as Gsequ
import Simulation.Simulator as Simu
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata


def estimator(SM,z,ut):
    Sigma,estimates = KF_est.KF_estimator(SM,z)
    ut_zt,Sigma_ut_zt = BA.Bayesian_analysis(SM,Sigma,estimates,z)
    H = SM[2]
    B = SM[1]
    ut = [np.dot(H,B)*ut[0],np.dot(H,B)*ut[1]]
    u_D = DM.Decision_making(ut,ut_zt,Sigma_ut_zt)
    return u_D

if __name__ == '__main__':
    dx = 1
    dz = 1
    # system matrices parameters
    """
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
    SM = [A,B,H,Q,R]
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
    Sigma,estimates = KF_est.KF_estimator(SM,z)
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    Sigma = cnvdata.convert_array2list_nd(Sigma,dx)
    
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)

    #u_D = estimator(SM,z,ut)
    