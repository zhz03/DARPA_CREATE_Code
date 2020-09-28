# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 00:44:31 2020

@author: Zhaoliang
"""

import numpy as np
import Simulation.Generate_seq_u as Gsequ
import Simulation.Simulator as Simu
import Estimator.KF_estimator as KF_est
import Estimator.Bayesian_analysis as BA
import Estimator.Decision_making as DM
import Estimator.estimator as Estr
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import Simulation.Binary_stat_hypothesis_testing_1d as BstatHT1d

def simulation(SM,x0,uts,ts,ut,trials):
    u_T_D = []
    for i in range(trials):
        u = Gsequ.generate_sequential_ut(uts,ts)
        y,z = Simu.Simulator(SM,x0,u)
        u_D = Estr.estimator(SM,z,ut)
        u_T = u[-1]
        u_td = [u_T,u_D]
        u_T_D.append(u_td)
    return u_T_D

if __name__ == '__main__':
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
    
    dx = 1
    dz = 1
    q = .1254
    r = 1.6798
    r = .1254
    a = 1.4816
    #a = 1.0
    h = 1.5927
    h = 1
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
    u_D = Estr.estimator(SM,z,ut)
    u_T_D1 = simulation(SM,x0,uts,ts,ut,trials)
    
    Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat = BstatHT1d.Bin_stat_hyp_test_1d(u_T_D1)
    

