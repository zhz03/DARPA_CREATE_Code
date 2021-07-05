# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:30:22 2020

@author: Zhaoliang and Zida 
"""
import numpy as np
import matplotlib.pyplot as plt


def uPlot(u, u_estimates):

    plt.plot(range(len(u)), u)
    plt.plot(range(len(u_estimates)), np.array(u_estimates).reshape(-1,))

    plt.legend(['u_gt','u_estimates'], loc='upper left')
    plt.title("RKF: U estimation")
    plt.ylabel('Positive Probability')
    plt.xlabel('Different System Models')
    plt.figure()
    plt.show()
    

def resultPlt(mode_num, SM_num, trials, prob_d_list, prob_f_list, prob_d_error_list):
 
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




if __name__ == '__main__':
    verification(2)
  
