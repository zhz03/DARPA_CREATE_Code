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

def estimator(SM,z,ut):
    Sigma,estimates = KF_est.KF_estimator(SM,z)
    ut_zt,Sigma_ut_zt = BA.Bayesian_analysis(SM,Sigma,estimates,z)
    u_D = DM.Decision_making(ut,ut_zt,Sigma_ut_zt)
    return u_D

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
    SM = [A,B,H,Q,R]
    u = Gsequ.generate_sequential_ut(uts,ts)
    y,z = Simu.Simulator(SM,x0,u)
    u_D = estimator(SM,z,ut)
    