# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 20:58:25 2020

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

def simulation_validation(A,B,H,Q,R,x0,uts,ts,sam_numb,flag=1):
    """
    This function is to validate the error probabilities by using  
    statistical data
    """
    if flag == 1:
        Uz_H0,Uz_H1 = E2Ef.Stat_Uz_H01(A,B,H,Q,R,x0,uts,ts,sam_numb)
    elif flag == 2:
        Uz_H1 = E2Ef.Stat_Uz(A,B,H,Q,R,x0,uts,ts,sam_numb)
        uts0 = [uts[0]]
        ts0 = [np.sum(ts)]
        Uz_H0 = E2Ef.Stat_Uz(A,B,H,Q,R,x0,uts0,ts0,sam_numb)
    
    Prob_D,Prob_FA,Prob_M,Prob_CR  = E2Ef.Bi_stat_HT_1d(Uz_H0,Uz_H1)
    
    return Prob_D,Prob_FA,Prob_M,Prob_CR

if __name__ == "__main__":
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
    Prob_D_stat,Prob_FA_stat,Prob_M_stat,Prob_CR_stat = simulation_validation(A,B,H,Q,R,x0,uts,ts,sam_numb)     