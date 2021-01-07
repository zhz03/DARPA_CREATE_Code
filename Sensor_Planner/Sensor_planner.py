# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:08:43 2020

@author: Zhaoliang
"""

import numpy as np

import Sensor_Planner.KF_planner_nd as kfplnnd
import Sensor_Planner.Ut_planner_nd as Uplnnd
import Sensor_Planner.Binary_hypothesis_testing_1d as Bht1d
from Sensor_Planner.mul_hypothesis_testing_nd import mul_Hyp_test_nd as mhtnd


from multi_HT_nd.input_generator_nd import input_generator_nd as InGen_nd
from E2Etest.SM_generator_nd import SM_generator_nd_single as SMGndsin

def Sensor_planner_1d(SM,T,ut):
    Sigma_xhat = kfplnnd.KF_planner(SM,T)
    Sigma_ut = Uplnnd.Ut_planner_nd(SM,Sigma_xhat)
    Sigma_uts = [Sigma_ut,Sigma_ut]
    H = SM[2]
    B = SM[1]
    ut = [np.dot(H,B)*ut[0],np.dot(H,B)*ut[1]]
    Prob_D,Prob_FA,Prob_M,Prob_CR = Bht1d.Bin_Hyp_test_1d(Sigma_uts,ut)
    return Prob_D,Prob_FA,Prob_M,Prob_CR    

def Sensor_planner_nd(SM,T,ut):
    Sigma_xhat = kfplnnd.KF_planner(SM,T)
    Sigma_ut = Uplnnd.Ut_planner_nd(SM,Sigma_xhat)    
    H = SM[2]
    B = SM[1]
    
    n_Hy = len(ut)
    #n_dim = np.size(ut[0])
    ut_hats = []
    for i in range(n_Hy):
        ut_hat = np.dot(H,B) * ut[i]
        ut_hats.append(ut_hat)
    sam_size = 1000    
    Prob_errors = mhtnd(ut_hats,Sigma_ut,sam_size)    
    return Sigma_ut
    
if __name__ == '__main__':
    # system model
    """
    dx = 1
    dz = 1
    A = np.array([1]).reshape(dx, dx)
    H = np.array([.1]).reshape(dz, dx)
    B = np.array([1]).reshape(dx, 1)
    Q = np.array([0.3 * 0.3]).reshape(dx, dx)
    R = np.array([0.2 * 0.2]).reshape(dz, dz)
    
    dx = 1
    dz = 1
    q = .1254
    r = 1.6798
    a = 1.4816
    h = 1.5927
    b = 1
    dx = 1
    dz = 1
    q = 0.1 * 0.1
    r = 1.0
    a = 1
    h = 40
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q]).reshape(dx, dx)
    R = np.array([r]).reshape(dz, dz)
    # timestep
    T = 100
    # ut
    ut = [0,1]
    SM = [A,B,H,Q,R]
    Sigma_xhat = kfplnnd.KF_planner(SM,T)
    Sigma_ut = Uplnnd.Ut_planner_nd(SM,Sigma_xhat)
    Sigma_uts = [Sigma_ut,Sigma_ut]
    H = SM[2]
    B = SM[1]
    ut = [np.dot(H,B)*ut[0],np.dot(H,B)*ut[1]]
    Prob_D,Prob_FA,Prob_M,Prob_CR = Bht1d.Bin_Hyp_test_1d(Sigma_uts,ut)
    """
    #Prob_D,Prob_FA,Prob_M,Prob_CR = Sensor_planner_1d(SM,T,ut)

    nHy = 3
    nd = 2
    Range = [0,5]
    means,Sigmas = InGen_nd(nHy,nd,Range)
    
    
    
    
   
    n_Hy = len(means)
    n_dim = np.size(means[0])
    
    Arange = [0,1]
    Brange = [0,1]
    Hrange = [0,1]
    Qrange = [0,1]    
    Rrange = [0,1]
    sm = SMGndsin(n_dim,n_dim,n_dim,Arange,Brange,Hrange,Qrange,Rrange)
    
    H = sm[2]
    B = sm[1]    
    ut = np.dot(H,B) 
    ut1 = np.dot(ut, means[0].reshape(n_dim))
