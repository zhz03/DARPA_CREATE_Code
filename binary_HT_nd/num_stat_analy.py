# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 12:34:08 2020

@author: Zhaoliang
"""

import numpy as np
import binary_HT_nd.sampling_binGDnd_Plot as smpl_bGDnd_plt 
import utility_functions.CompP2SHist as CompP2S
import matplotlib.pyplot as plt  

def num_stat_analy(filepath):
    
    Mean0_norms = []
    Mean1_norms = []
    Var0_norms = []
    Var1_norms = []
    
    Error_mean0,Error_mean1,Error_var0,Error_var1 = smpl_bGDnd_plt.load_stat_data(filepath)
    
    num = len(Error_mean0)
    
    for i in range(num):
        mean0_norm = np.linalg.norm(Error_mean0[i])
        mean1_norm = np.linalg.norm(Error_mean1[i])
        var0_norm = np.linalg.norm(Error_var0[i])
        var1_norm = np.linalg.norm(Error_var1[i])
        Mean0_norms.append(mean0_norm)
        Mean1_norms.append(mean1_norm)
        Var0_norms.append(var0_norm)
        Var1_norms.append(var1_norm)
    return Mean0_norms,Mean1_norms,Var0_norms,Var1_norms

def save_plot_results(filepath):
    Mean0_norms,Mean1_norms,Var0_norms,Var1_norms = num_stat_analy(filepath)
    fig_path = './figs/stat_2d/'

    compp2s = CompP2S.Compare_pln2statis_hist(mean_pln = None, Sigma_pln = None,name ='plan',bins = None,Range=0.2)
    
    figname = 'Mean0_norms'
    compp2s.visualization_self(Mean0_norms,nflg = False,dataname = figname ,tname = "The histogram of error norm")
    fig_name1= fig_path + figname + '.jpg'
    plt.savefig(fig_name1)
    
    plt.figure()
    figname = 'Mean1_norms'
    compp2s.visualization_self(Mean1_norms,nflg = False,dataname = figname,tname = "The histogram of error norm")
    fig_name2= fig_path + figname + '.jpg'
    plt.savefig(fig_name2)
    
    plt.figure()
    figname = 'Var0_norms'
    compp2s.visualization_self(Var0_norms,nflg = False,dataname = figname,tname = "The histogram of error norm")
    fig_name3= fig_path + figname + '.jpg'
    plt.savefig(fig_name3)    
    
    plt.figure()
    figname = 'Var1_norms'
    compp2s.visualization_self(Var1_norms,nflg = False,dataname =figname,tname = "The histogram of error norm")
    fig_name4= fig_path + figname + '.jpg'
    plt.savefig(fig_name4)        
    
if __name__ == "__main__":
    filepath = './data_storage/verification_2d/'
    """
    Error_mean0,Error_mean1,Error_var0,Error_var1 = smpl_bGDnd_plt.load_stat_data(filepath)    
    Mean0_norms,Mean1_norms,Var0_norms,Var1_norms = num_stat_analy(Error_mean0,Error_mean1,Error_var0,Error_var1)
    """
    save_plot_results(filepath)
   