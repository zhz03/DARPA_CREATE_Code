# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:08:43 2020

@author: Zhaoliang
"""

import numpy as np

import Sensor_Planner.KF_planner_nd as kfplnnd
import Sensor_Planner.Ut_planner_nd as Uplnnd
import Sensor_Planner.Binary_hypothesis_testing_1d as Bht1d

def Sensor_planner_1d(A,B,H,Q,R,T,ut):
    Sigma_xhat = kfplnnd.KF_planner(A, B, H, Q, R,T)
    Sigma_ut = Uplnnd.Ut_planner_nd(A, B, H, Q, R,Sigma_xhat)
    Sigma_uts = [Sigma_ut,Sigma_ut]
    Prob_D,Prob_FA,Prob_M,Prob_CR = Bht1d.Bin_Hyp_test_1d(Sigma_uts,ut)
    return Prob_D,Prob_FA,Prob_M,Prob_CR    
if __name__ == '__main__':
    # system model
    dx = 1
    dz = 1
    A = np.array([1]).reshape(dx, dx)
    H = np.array([1]).reshape(dz, dx)
    B = np.array([1]).reshape(dx, 1)
    Q = np.array([0.5 * 0.5]).reshape(dx, dx)
    R = np.array([0.5 * 0.5]).reshape(dz, dz)
    # timestep
    T = 100
    # ut
    ut = [0,1]
    Prob_D,Prob_FA,Prob_M,Prob_CR = Sensor_planner_1d(A,B,H,Q,R,T,ut)

    
