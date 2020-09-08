# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 20:00:18 2020

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

def Sensor_planner(A,B,H,Q,R,uts,ts):
    """
    This function outputs sensor planning error probabilities
    """
    USigma0 = kfpln.KF_planning_Uz(A, B, H, Q, R,ts[0])
    USigma1 = kfpln.KF_planning_Uz(A, B, H, Q, R,ts[0])
    mean_1 = np.dot(H,B)*uts[1]
    mean_0 = np.dot(H,B)*uts[0]

    Prob_D,Prob_FA,Prob_M,Prob_CR = Gd.error_prob(mean_0,mean_1,USigma0,USigma1)
    return Prob_D,Prob_FA,Prob_M,Prob_CR

if __name__ == "__main__":
    dx = 1
    dz = 1
    A = np.array([1]).reshape(dx, dx)
    H = np.array([1]).reshape(dz, dx)
    B = np.array([1]).reshape(dx, 1)
    Q = np.array([0.5 * 0.5]).reshape(dx, dx)
    R = np.array([0.5 * 0.5]).reshape(dz, dz)
    # here 0 stands for no event happens, 1 stands for event happens 
    uts = [0,1] 
    # a particular timestep. 
    # we need to use the planner to determine the error probabilities at that time step.
    ts = [100]
    
    Prob_D,Prob_FA,Prob_M,Prob_CR = Sensor_planner(A,B,H,Q,R,uts,ts)    
    
    