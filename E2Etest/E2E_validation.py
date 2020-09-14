# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:30:22 2020

@author: Zhaoliang
"""
import numpy as np
import Sensor_Planner.Sensor_planner as Spr
import Simulation.Binary_stat_hypothesis_testing_1d as BstatHT1d
import Simulation.Simulation as Sim
import E2Etest.Error_prob_comparison as EPComp

def E2E_validation(SM,T,ut,x0,uts,ts,trials):

    Pr_D,Pr_FA,Pr_M,Pr_CR = Spr.Sensor_planner_1d(SM,T,ut)
    u_T_D = Sim.simulation(SM,x0,uts,ts,ut,trials)
    Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat = BstatHT1d.Bin_stat_hyp_test_1d(u_T_D)
    error_D,error_FA,error_M,error_CR = EPComp.Error_prob_Comp(Pr_D,Pr_FA,Pr_M,Pr_CR,Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat)
    return error_D,error_FA,error_M,error_CR
    
if __name__ == '__main__':
    dx = 1
    dz = 1
    # system matrices parameters
    q = 2.0
    r = 1.0
    a = 1
    h = 1
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    SM = [A,B,H,Q,R]
    x0 = np.array([[0]]).reshape(dx, 1)
    uts = [0,1]
    ts = [100,1]
    ut = [0,1] 
    trials = 1000
    T = 101
    
    #error_D,error_FA,error_M,error_CR = E2E_validation(SM,T,ut,x0,uts,ts,trials)

    Pr_D,Pr_FA,Pr_M,Pr_CR = Spr.Sensor_planner_1d(SM,T,ut)
    u_T_D = Sim.simulation(SM,x0,uts,ts,ut,trials)
    Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat = BstatHT1d.Bin_stat_hyp_test_1d(u_T_D)
    error_D,error_FA,error_M,error_CR = EPComp.Error_prob_Comp(Pr_D,Pr_FA,Pr_M,Pr_CR,Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat)
    