# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:15:26 2020

@author: Zhaoliang and Zida 
"""

import numpy as np
import Simulations.Simulation as Sim
import utility_functions.CompP2SHist as CompP2SHist
import random
import matplotlib.pyplot as plt

def Bin_stat_hyp_test_1d(u_T_D):
    num = len(u_T_D)
    count = 0
    for i in range(num):
        # It's right only u_T_D[i][0] = 1 (as we set ut[-1] always = 1 )
        if u_T_D[i][0]==u_T_D[i][1]:  
            count = count + 1
    Prob_D_stat = count/num
    Prob_FA_stat = (num-count)/num
    Prob_M_stat = 1 - Prob_D_stat
    Prob_CR_stat = 1 - Prob_FA_stat
    return Prob_D_stat,Prob_FA_stat,Prob_M_stat,Prob_CR_stat

def Bin_stat_hyp_test_nd(u_T_D, ut):
    num = len(u_T_D)
    len_ut = len(ut)
    Prob_error_stat = np.zeros([len_ut, len_ut])
    Prob_error = np.zeros([len_ut, len_ut])
    u_gt = np.zeros(len_ut)
    u_est = np.zeros(len_ut)
    count = 0
    for i in range(num):
        for j in range(len_ut):
            if (u_T_D[i][0] == ut[j]):
                gt_i = j
        for j in range(len_ut):
            if (u_T_D[i][1] == ut[j]):
                est_i = j
            
        Prob_error_stat[gt_i][est_i] += 1
        
    for i in range(len_ut):
        for j in range(len_ut):     
            if sum(Prob_error_stat[i]) != 0:
                   Prob_error[i][j] = Prob_error_stat[i][j] / sum(Prob_error_stat[i]) 
        
    return Prob_error

def verification(num):
    Error_D = []
    Error_FA = []
    Error_CR = []
    Error_M = []
    for i in range(num):
        samp_num = 1000
        u_T_D = []
        for j in range(samp_num):
            u_T = 1
            u_D = round(random.uniform(0,1))
            u_td = [u_T,u_D]
            u_T_D.append(u_td)
        Prob_D_stat,Prob_FA_stat,Prob_M_stat,Prob_CR_stat = Bin_stat_hyp_test_1d(u_T_D)
        Prob_uni = 0.5
        error_D = Prob_D_stat - Prob_uni
        error_FA = Prob_FA_stat - Prob_uni
        error_M = Prob_M_stat - Prob_uni
        error_CR = Prob_CR_stat - Prob_uni
        Error_D.append(error_D)
        Error_FA.append(error_FA)
        Error_M.append(error_M)
        Error_CR.append(error_CR)
    
    CompP2S = CompP2SHist.Compare_pln2statis_hist(Range = 0.1)
    CompP2S.visualization_self(Error_D,nflg = True)
    fig_name1 = './figs/Bin_stat_HT_figs/' + 'Error_D.jpg'
    plt.savefig(fig_name1) 
    plt.close()
    CompP2S.visualization_self(Error_FA,nflg = True)
    fig_name2 = './figs/Bin_stat_HT_figs/' + 'Error_FA.jpg'
    plt.savefig(fig_name2) 
    plt.close()
    CompP2S.visualization_self(Error_M,nflg = True)
    fig_name3 = './figs/Bin_stat_HT_figs/' + 'Error_M.jpg'
    plt.savefig(fig_name3) 
    plt.close()
    CompP2S.visualization_self(Error_CR,nflg = True)
    fig_name4 = './figs/Bin_stat_HT_figs/' + 'Error_CR.jpg'
    plt.savefig(fig_name4) 
    plt.close()    
if __name__ == '__main__':
    verification(1000)
    
    """
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
    """
    #Prob_D_stat,Prob_FA_stat,Prob_M_stat,Prob_CR_stat = Bin_stat_hyp_test_1d(u_T_D)