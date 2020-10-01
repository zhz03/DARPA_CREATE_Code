# -*- coding: utf-8 -*-
"""
Created on Thu Oct  1 16:19:03 2020

@author: Zhaoliang
"""

import numpy as np
import Simulations.Generate_seq_u as Gsequ
import Simulations.Simulator as Simu
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import utility_functions.CompP2SHist as CompP2SHist
import Estimator.Decision_making as DM
import E2Etest.SM_generator_1d as SMGen1d
import E2Etest.System_setup_generator as SSGen
import matplotlib.pyplot as plt
import Estimator.KF_estimator as KF

def verification(num):
    dx = 1
    Arange = [1,1]
    Brange = [1,1]
    Hrange = [0,2]
    Qrange = [0,2]    
    Rrange = [0,2]
    System_models = SMGen1d.SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange)
    #As,Hs,Bs,Qs,Rs,
    T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
    SM_num = len(System_models[0])
    T = 1000
    ts[0] = T
    for i in range(SM_num):
        A = System_models[0][i]
        B = System_models[1][i]
        H = System_models[2][i]
        Q = System_models[3][i]
        R = System_models[4][i]
        SM = [A,B,H,Q,R]
        u = Gsequ.generate_sequential_ut(uts,ts)
        y,z = Simu.Simulator(SM,x0,u)
        Sigma,estimates = KF.KF_estimator(SM,z)
        
        ground_truth = cnvdata.convert_array2list_nd(y,dx)
        measurements = cnvdata.convert_array2list_nd(z,dx) 
        estimates = cnvdata.convert_array2list_nd(estimates,dx)
        errors = [estimates[0][i] - ground_truth[0][i] for i in range(len(estimates[0]))]
        #predictions = cnvdata.convert_array2list_nd(predictions,dx)
        plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name = './figs/KF_estimator_figs/' + str(i) + '.jpg'
        plt.savefig(fig_name)
        plt.close()

        CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = 0, Sigma_pln = Sigma[-1][0][0])
        CompP2S.visualization_compare(errors,1)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name1 = './figs/KF_estimator_figs/' + str(i) + '_error.jpg'
        plt.savefig(fig_name1)
        plt.close()
    
if __name__ == '__main__':
    verification(20)    