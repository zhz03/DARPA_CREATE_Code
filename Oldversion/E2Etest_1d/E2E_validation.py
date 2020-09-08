# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 22:34:21 2020

@author: Zhaoliang
"""

import numpy as np

import Sim_KF_Pln_nd.simulation as sim1
import Sim_KF_Pln_nd.kalman_filter as kf
import Sim_KF_Pln_nd.planning as kfpln

import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import utility_functions.CompP2SHist as CompP2SHist

import binary_HT_1d.Gaussian_dist as Gd
import E2E_functions as E2Ef
import Sensor_planner as SP
import Simulation_validation as SV

def Error_prob_Comp(Pr_D,Pr_FA,Pr_M,Pr_CR,Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat):
    error_D = Pr_D_stat - Pr_D
    error_FA = Pr_FA_stat - Pr_FA
    error_M = Pr_M_stat - Pr_M
    error_CR = Pr_CR_stat - Pr_CR
    return error_D,error_FA,error_M,error_CR

def E2E_validation(A,B,H,Q,R,x0,uts,ts,sample_num):
    Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat = SV.simulation_validation(A,B,H,Q,R,x0,uts,ts,sample_num)
    Pr_D,Pr_FA,Pr_M,Pr_CR  = SP.Sensor_planner(A,B,H,Q,R,uts,ts)
    error_D,error_FA,error_M,error_CR = Error_prob_Comp(Pr_D,Pr_FA,Pr_M,Pr_CR,Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat)
    return error_D,error_FA,error_M,error_CR

if __name__ =="__main__":
    dx = 1
    dz = 1
    A = np.array([1]).reshape(dx, dx)
    H = np.array([1]).reshape(dz, dx)
    B = np.array([1]).reshape(dx, 1)
    Q = np.array([0.5 * 0.5]).reshape(dx, dx)
    R = np.array([0.5 * 0.5]).reshape(dz, dz)
    uts = [0,1]
    ts = [100,1]
    x0 = np.array([[0]]).reshape(dx, 1)
    sam_numb = 1000
    error_D,error_FA,error_M,error_CR = E2E_validation(A,B,H,Q,R,x0,uts,ts,sam_numb)
    