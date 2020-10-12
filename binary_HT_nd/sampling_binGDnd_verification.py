# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 14:46:45 2020

@author: Zhaoliang
"""
import numpy as np
import random
import utility_functions.CompP2SHist as CompP2S
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import matplotlib.pyplot as plt
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd

def savefigs(title,fig_path,i = None,close_flg = True):
    plt.title(title)
    if i == None:
        fig_name = fig_path + title + '.jpg'
    else:
        fig_name = fig_path + str(i) + '.jpg'
    plt.savefig(fig_name)
    if close_flg == True:
        plt.close()         
    
def verification_1d(range1):
    
    m0 = round(random.uniform(range1[0],range1[1]))
    m1 = round(random.uniform(range1[0],range1[1]))
    mean0 = np.array([m0])
    mean1 = np.array([m1])
    s0 = abs(round(random.uniform(range1[0],range1[1])))
    s1 = abs(round(random.uniform(range1[0],range1[1])))
    Sigma0 = np.array([s0]).reshape(1, 1)
    Sigma1 = np.array([s1]).reshape(1, 1)
    num_sam = 1000
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,num_sam)
    points0 = points[0:num_sam,:]
    points1 = points[num_sam:num_sam * 2,:]
    
    compp2s = CompP2S.Compare_pln2statis_hist(mean_pln = m0, Sigma_pln = s0,name ='plan',bins = None,Range=None)
    #compp2s0.visualization_compare(points0,10)
    #compp2s1 = CompP2S.Compare_pln2statis_hist(mean_pln = m1, Sigma_pln = s1,name ='plan',bins = None,Range=None)
    #compp2s0.visualization_compare(points1,10)
    
    plotfgs.plot_2_Gaussian(m0,m1,s0,s1)
    compp2s.visualization_self(points0,nflg = True,dataname ='stat0')
    compp2s.visualization_self(points1,nflg = True,dataname ='stat1')
    
    title = ''
    fig_path = './figs/verification_1d/'
    savefigs(title,fig_path,i = None,close_flg = False)
    
    mean_stat0,var_stat0 = compp2s.calculate_stat(points0)
    mean_stat1,var_stat1 = compp2s.calculate_stat(points1)
    error_mean0 = m0 - mean_stat0
    error_var0 = s0 - var_stat0
    error_mean1 = m1 - mean_stat1
    error_var1 = s1 - var_stat1
    
    return error_mean0,error_mean1,error_var0,error_var1
    
"""    
def sampling_binGDnd_verification(num,dim_type):
    if dim_type == 1:
"""

if __name__ == "__main__":
    range1 = [-10,10]
    error_mean0,error_mean1,error_var0,error_var1 = verification_1d(range1)