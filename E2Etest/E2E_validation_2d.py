# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:30:22 2020

@author: Zhaoliang and Zida 
"""
import numpy as np
import Sensor_Planner.Sensor_planner as Spr
import Simulations.Binary_stat_hypothesis_testing_1d as BstatHT
import Simulations.Simulation as Sim
import E2Etest.Error_prob_comparison as EPComp
import E2Etest.SM_generator_1d as SMGen1d
import E2Etest.SM_generator_nd as SMGennd
import E2Etest.System_setup_generator as SSGen
import matplotlib.pyplot as plt
import E2Etest.E2E_plot as E2Eplt
from enum import Enum

class Mode_simulation(Enum):
    raw = 1 
    MM = 2 #MM
    raw_ugt = 3 #groudtruth
    rkf = 4

def E2E_validation_1d(SM,T,ut,x0,uts,ts,trials):

    Pr_D,Pr_FA,Pr_M,Pr_CR = Spr.Sensor_planner_1d(SM,T,ut)
    # Prob_error = Spr.Sensor_planner_nd(SM,T,ut)
    # Pr_D = Prob_error[0][0] 
    # Pr_FA = Prob_error[0][1] 
    # Pr_M = Prob_error[1][0] 
    # Pr_CR = Prob_error[1][1] 
    u_T_D = Sim.simulation(SM,x0,uts,ts,ut,trials)
    Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat = BstatHT.Bin_stat_hyp_test_1d(u_T_D)
    
    print("Sensor Planner result: ", Pr_D,Pr_FA,Pr_M,Pr_CR)
    print("Statistical result : ",Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat)
    
    error_D,error_FA,error_M,error_CR = EPComp.Error_prob_Comp(Pr_D,Pr_FA,Pr_M,Pr_CR,Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat)
    return error_D,error_FA,error_M,error_CR

def E2E_validation_nd(SM,T,ut,x0,uts,ts,trials):

    Prob_error = Spr.Sensor_planner_nd(SM,T,ut)
    u_T_D = Sim.simulation(SM,x0,uts,ts,ut,trials)
    Prob_error_stat = BstatHT.Bin_stat_hyp_test_nd(u_T_D, ut)
    error_comparison = EPComp.Error_prob_Comp_nd(Prob_error,Prob_error_stat)
    return Prob_error, Prob_error_stat, error_comparison 

def E2E_validation_nd_with_mode(SM,T,ut,x0,uts,ts,trials,mode_simulation):

    Prob_error = Spr.Sensor_planner_nd(SM,T,ut)
    Prob_error_stat_list = []
    error_comparison_list = []
    
    u_T_D_list = Sim.simulation_with_mode(SM,x0,uts,ts,ut,trials,mode_simulation)

    for i in range(len(u_T_D_list)): # only for raw/MM/ugt
        Prob_error_stat = BstatHT.Bin_stat_hyp_test_nd(u_T_D_list[i], ut)
        error_comparison = EPComp.Error_prob_Comp_nd(Prob_error,Prob_error_stat)
        Prob_error_stat_list.append(Prob_error_stat)
        error_comparison_list.append(error_comparison)
        
    return Prob_error, Prob_error_stat_list, error_comparison_list

if __name__ == '__main__':

    Arange = [-1,1]
    Brange = [0,2]
    Hrange = [0,2]
    Qrange = [0,2]    
    Rrange = [0,2]
    
    mode = "const_2d"    
    mode_simulation = Mode_simulation # Four mode: raw & MM & raw_ugt & rkf
    model_num = 2
    trials_ = 100 
    nHy = 2      
    nd = 2
    dx = nd
    dz = nd 
    Ts_ = 20
    mode_num = 4        
        
    if mode == "1d":
        System_models = SMGen1d.SM_generator_1d(model_num,Arange,Brange,Hrange,Qrange,Rrange)
        T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
 
    elif mode == "nd":              
        du = 1
        MRange = [0,nHy]
        System_models = SMGennd.SM_generator_nd(dx,dz,du,Arange,Brange,Hrange,Qrange,Rrange,model_num)
        T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator_nd(nHy,nd,du,MRange,Ts_,trials_)
   
    elif mode == "const_2d":   #read ABHQR from .npy
        du = 1
        MRange = [0,nHy]
        System_models = SMGennd.SM_generator_constant(Qrange,Rrange,model_num)
        T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator_nd(nHy,nd,du,MRange,Ts_,trials_)
        
    SM_num = len(System_models[0])  #SM = [As,Bs,Hs,Qs,Rs]; As = []
    prob_error_list = []
    prob_error_stat_list_raw = []
    error_comparison_list = []
    prob_error_stat_list_MM = []
    error_comparison_list_MM = []
    prob_error_stat_list_ugt = []
    error_comparison_list_ugt = []
    
    for i in range(SM_num):
        A = System_models[0][i]
        B = System_models[1][i]
        H = System_models[2][i]
        Q = System_models[3][i]
        R = System_models[4][i]
        SM = [A,B,H,Q,R]
        
        if mode=="1d":
            error_D,error_FA,error_M,error_CR = E2E_validation_1d(SM,T,ut,x0,uts,ts,trials)
            
        elif mode=="nd" or mode=="const_2d":
            Prob_error, Prob_error_stat, error_comparison  = E2E_validation_nd_with_mode(SM,T,ut,x0,uts,ts,trials,mode_simulation)
            
            prob_error_list.append(Prob_error)
            prob_error_stat_list_raw.append(Prob_error_stat[0])
            error_comparison_list.append(error_comparison[0])
            prob_error_stat_list_MM.append(Prob_error_stat[1])
            error_comparison_list_MM.append(error_comparison[1])
            prob_error_stat_list_ugt.append(Prob_error_stat[2])
            error_comparison_list_ugt.append(error_comparison[2])
    
    if mode=="nd" or mode=="const_2d":     
   
        prob_d_list = np.zeros([mode_num, SM_num]) #4: sim; raw; MM; ugt
        prob_f_list = np.zeros([mode_num, SM_num]) #4: sim; raw; MM; ugt
        prob_d_error_list = np.zeros([mode_num-1, SM_num]) #3: raw; MM; ugt
        prob_f_error_list = np.zeros([mode_num-1, SM_num]) #3: raw; MM; ugt    
        
        for i in range(SM_num):
            len_ut = prob_error_list[i].shape[0]
            for j in range(len_ut):
                prob_d_list[0][i] += prob_error_list[i][j][j]/2
                prob_d_list[1][i] += prob_error_stat_list_raw[i][j][j]
                prob_d_list[2][i] += prob_error_stat_list_MM[i][j][j]
                prob_d_list[3][i] += prob_error_stat_list_ugt[i][j][j]
                    
                for k in range(len_ut):
                    if k != j:
                        prob_f_list[0][i] += prob_error_list[i][j][k]
                        prob_f_list[1][i] += prob_error_stat_list_raw[i][j][k]
                        prob_f_list[2][i] += prob_error_stat_list_MM[i][j][k]
                        prob_f_list[3][i] += prob_error_stat_list_ugt[i][j][k]
                                                      
        for i in range(mode_num-1):
            prob_d_error_list[i] = prob_d_list[0] - prob_d_list[i+1]
            prob_f_error_list[i] = prob_f_list[0] - prob_f_list[i+1]
        if mode_num == 4:  
            print("mean error comparison is {}\n", np.mean(prob_d_error_list[0]), \
                  "mean error comparison of MM is {}\n", np.mean(prob_d_error_list[1]),\
                  "mean error comparison of ugt is {}\n", np.mean(prob_d_error_list[2]))
        else:
            print("mean error comparison is {}\n", np.mean(prob_d_error_list[0]))
 
        E2Eplt.resultPlt(mode_num, SM_num, trials, prob_d_list, prob_f_list, prob_d_error_list)