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
from enum import Enum

class Mode_simulation(Enum):
    both = 0
    raw = 1 
    MM = 2 #MM
    raw_ugt = 3 #groudtruth
    
def E2E_validation_1d(SM,T,ut,x0,uts,ts,trials):

    Pr_D,Pr_FA,Pr_M,Pr_CR = Spr.Sensor_planner_1d(SM,T,ut)
    
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
    if mode_simulation.value == Mode_simulation.both.value:
        Prob_error_stat_list = []
        error_comparison_list = []
        
        # u_T_D, u_T_D_MM, u_T_D_ugt = Sim.simulation_with_mode(SM,x0,uts,ts,ut,trials,mode_simulation)
        
        u_T_D_list = Sim.simulation_with_mode(SM,x0,uts,ts,ut,trials,mode_simulation)

        for i in range(len(u_T_D_list)):
            Prob_error_stat = BstatHT.Bin_stat_hyp_test_nd(u_T_D_list[i], ut)
            error_comparison = EPComp.Error_prob_Comp_nd(Prob_error,Prob_error_stat)
            Prob_error_stat_list.append(Prob_error_stat)
            error_comparison_list.append(error_comparison)
            
        # Prob_error_stat = BstatHT.Bin_stat_hyp_test_nd(u_T_D_list[0], ut)
        # error_comparison = EPComp.Error_prob_Comp_nd(Prob_error,Prob_error_stat)
        # Prob_error_stat_list.append(Prob_error_stat)
        # error_comparison_list.append(error_comparison)
        
        # Prob_error_stat_MM = BstatHT.Bin_stat_hyp_test_nd(u_T_D_list[1], ut)
        # error_comparison_MM = EPComp.Error_prob_Comp_nd(Prob_error,Prob_error_stat_MM)
        # Prob_error_stat_list.append(Prob_error_stat_MM)
        # error_comparison_list.append(error_comparison_MM)
        
        # Prob_error_stat_ugt = BstatHT.Bin_stat_hyp_test_nd(u_T_D_list[2], ut)
        # error_comparison_ugt = EPComp.Error_prob_Comp_nd(Prob_error,Prob_error_stat_ugt)
        # Prob_error_stat_list.append(Prob_error_stat_ugt)
        # error_comparison_list.append(error_comparison_ugt)
        
        return Prob_error, Prob_error_stat_list, error_comparison_list
        
    else:
        u_T_D = Sim.simulation_with_mode(SM,x0,uts,ts,ut,trials,mode_simulation)
        Prob_error_stat = BstatHT.Bin_stat_hyp_test_nd(u_T_D, ut)
        error_comparison = EPComp.Error_prob_Comp_nd(Prob_error,Prob_error_stat)
        
        return Prob_error, Prob_error_stat, error_comparison 

if __name__ == '__main__':

    Arange = [-1,1]
    Brange = [0,2]
    Hrange = [0,2]
    Qrange = [0,2]    
    Rrange = [0,2]
    
    mode = "const_2d"    
    mode_simulation = Mode_simulation.both # Four mode: raw & MM & raw_ugt & both
    num = 3
    trials_ = 100
    nHy = 2      
    nd = 2
    dx = nd
    dz = nd 
    Ts_ = 10
    mode_num = 4
        
    if mode_simulation.value == Mode_simulation.both.value:
        mode_num = 4
    else:
        mode_num = 2 
        
    if mode == "1d":
        System_models = SMGen1d.SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange)
        T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
 
    elif mode == "nd":              
        du = 1
        MRange = [0,nHy]
        System_models = SMGennd.SM_generator_nd(dx,dz,du,Arange,Brange,Hrange,Qrange,Rrange,num)
        T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator_nd(nHy,nd,du,MRange,Ts_,trials_)
   
    elif mode == "const_2d":   #read ABHQR from .npy
        du = 1
        MRange = [0,nHy]
        System_models = SMGennd.SM_generator_constant(Qrange,Rrange,num)
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
                if mode_simulation == Mode_simulation.both:
                    prob_d_list[2][i] += prob_error_stat_list_MM[i][j][j]
                    prob_d_list[3][i] += prob_error_stat_list_ugt[i][j][j]
                    
                for k in range(len_ut):
                    if k != j:
                        prob_f_list[0][i] += prob_error_list[i][j][k]
                        prob_f_list[1][i] += prob_error_stat_list_raw[i][j][k]
                        if mode_simulation == Mode_simulation.both:
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
            
        '''
        Plot Section
        '''    
        for i in range(mode_num):
            plt.plot(range(SM_num), prob_d_list[i])
        if mode_num == 4:
            plt.legend(['Planner','Simulation_raw','Simulation_MM','Simulation_ugt'], loc='upper left')
        else:
            plt.legend(['Planner','Simulation'], loc='upper left')
        plt.title("Sensor Detection Error bound with {} trials/Iteration: Prob_D".format(trials))
        plt.ylabel('Positive Probability')
        plt.xlabel('Different System Models')
        plt.figure()
        plt.show()
        
        # for i in range(mode_num):
        #     plt.plot(range(SM_num), prob_f_list[i])
            
        # if mode_num == 4:
        #     plt.legend(['Planner','Simulation_raw','Simulation_MM','Simulation_ugt'], loc='upper left')
        # else:
        #     plt.legend(['Planner','Simulation'], loc='upper left')
        # plt.title("Sensor Detection Error bound with {} trials/Iteration: Prob_F".format(trials))
        # plt.ylabel('Flase Probability')
        # plt.xlabel('Different System Models')
        # plt.legend(['Planner','Simulation_raw'], loc='upper left')
        # plt.figure()
        # plt.show()
        
        for i in range(mode_num-1):
            plt.plot(range(SM_num), prob_d_error_list[i])     
        if mode_num == 4:
            plt.legend(['Prob_d Diff_raw','Prob_d Diff_MM','Prob_d Diff_ugt'], loc='upper left')
        else:
            plt.legend(['Prob_d Diff'], loc='upper left')
        plt.title("Sensor Detection Error Difference with {} trials/Iteration".format(trials))
        plt.ylabel('Error Probability Diff')
        plt.xlabel('Different System Models')
        plt.figure()
        plt.show()
        
        #plt.axhline(y=np.mean(prob_d_error_list[0]), color='g', linestyle='-')
        #plt.axhline(y=np.mean(prob_d_error_list[1]), color='b', linestyle='-')
    
        plt.bar(np.arange(SM_num)-0.2, prob_d_list[1] , alpha=0.8, width=0.2, color='blue', label='Raw Estimator', lw=4)
        plt.bar(np.arange(SM_num), prob_d_list[2], alpha=0.9, width=0.2, color='green', label='MM Estimator', lw=4)
        plt.bar(np.arange(SM_num)+0.2, prob_d_list[3], alpha=0.9, width=0.2, color='red', label='UGT Estimator', lw=4)
        plt.ylabel('Positive Probability', fontsize=15)
        plt.xlabel('Different System Models', fontsize=15)
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        plt.legend(loc='upper left',fontsize=15)
        plt.show()