# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:08:43 2020

@author: Zhaoliang
"""

import numpy as np

import Sensor_Planner.KF_planner_nd as kfplnnd
import Sensor_Planner.Ut_planner_nd as Uplnnd
import Sensor_Planner.Binary_hypothesis_testing_1d as Bht1d

def Sensor_planner_1d(SM,T,ut):
    Sigma_xhat = kfplnnd.KF_planner(SM,T)
    Sigma_ut = Uplnnd.Ut_planner_nd(SM,Sigma_xhat)
    Sigma_uts = [Sigma_ut,Sigma_ut]
    H = SM[2]
    B = SM[1]
    ut = [np.dot(H,B)*ut[0],np.dot(H,B)*ut[1]]
    Prob_D,Prob_FA,Prob_M,Prob_CR = Bht1d.Bin_Hyp_test_1d(Sigma_uts,ut)
    return Prob_D,Prob_FA,Prob_M,Prob_CR    

if __name__ == '__main__':
    # system model
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
    
    #Prob_D,Prob_FA,Prob_M,Prob_CR = Sensor_planner_1d(SM,T,ut)

    
