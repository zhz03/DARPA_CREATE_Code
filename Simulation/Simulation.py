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

def simulation(A,B,H,Q,R,x0,uts,ts,ut,trials):
    u_T_D = []
    for i in range(trials):
        u = Gsequ.generate_sequential_ut(uts,ts)
        y,z = Simu.Simulator(A,B,H,Q,R,x0,u)
        u_D = Estr.estimator(A,B,H,Q,R,z,ut)
        u_T = u[-1]
        u_td = [u_T,u_D]
        u_T_D.append(u_td)
    return u_T_D

if __name__ == '__main__':
    dx = 1
    dz = 1
    # system matrices parameters
    q = 0.5
    r = 0.5
    a = 1
    h = 1
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    uts = [0,1]
    ts = [100,10]
    ut = [0,1] 
    trials = 100
    u_T_D = simulation(A,B,H,Q,R,x0,uts,ts,ut,trials)
    

