# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:15:26 2020

@author: Zhaoliang
"""

import numpy as np
import Simulations.Simulation as Sim

def Bin_stat_hyp_test_1d(u_T_D):
    num = len(u_T_D)
    count = 0
    for i in range(num):
        if u_T_D[i][0]==u_T_D[i][1]:
            count = count + 1
    Prob_D_stat = count/num
    Prob_FA_stat = (num-count)/num
    Prob_M_stat = 1 - Prob_D_stat
    Prob_CR_stat = 1 - Prob_FA_stat
    return Prob_D_stat,Prob_FA_stat,Prob_M_stat,Prob_CR_stat

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
    trials = 1000
    SM = [A,B,H,Q,R]
    u_T_D = Sim.simulation(SM,x0,uts,ts,ut,trials)
    Prob_D_stat,Prob_FA_stat,Prob_M_stat,Prob_CR_stat = Bin_stat_hyp_test_1d(u_T_D)