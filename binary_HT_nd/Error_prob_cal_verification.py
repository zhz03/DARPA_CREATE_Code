# -*- coding: utf-8 -*-
"""
Created on Wed Oct 14 12:10:17 2020

@author: Zhaoliang
"""

import numpy as np
import random
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd
import binary_HT_nd.Error_prob_cal as EprC
import binary_HT_nd.sampling_binGDnd_Plot as smpl_bGDnd_plt
import matplotlib.pyplot as plt
import utility_functions.convert_data as cnvdata
from scipy.stats import norm, multivariate_normal

def visualization_2d(points_D,points_FA,points_M,points_CR):
    points_D_con = np.array(points_D)
    points_FA_con = np.array(points_FA)
    points_M_con = np.array(points_M)
    points_CR_con = np.array(points_CR)

    plt.plot(points_D_con[:,0],points_D_con[:,1],'ro',label='Pr_D')
    plt.plot(points_CR_con[:,0],points_CR_con[:,1],'bo',label='Pr_CR')
    plt.plot(points_FA_con[:,0],points_FA_con[:,1],'yo',label='Pr_FA')
    plt.plot(points_M_con[:,0],points_M_con[:,1],'ko',label='Pr_M')
    plt.legend()
    plt.show()

def Epr_verification_2d(plt_fig):
    filepath = './data_storage/verification_2d/'
    Sample_points,M0,M1,S0,S1 = smpl_bGDnd_plt.load_plt_data(filepath)
    
    trial_num = len(M0)
    if plt_fig == -1:
        for i in range(trial_num):
            points = Sample_points[i]
            m0 = M0[i]
            m1 = M1[i]
            s0 = S0[i]
            s1 = S1[i]
            Pr_D,Pr_M,Pr_FA,Pr_CR,Points_D,Points_M,Points_FA,Points_CR = EprC.Error_prob_cal(m0,m1,s0,s1,points,True)
            visualization_2d(Points_D,Points_FA,Points_M,Points_CR)
            
            title = 'Error_prob_cal Verification for data ' + str(i) 
            fig_path = './figs/error_prob_verification_2d/'
            smpl_bGDnd_plt.savefigs(title,fig_path,i = None,close_flg = True)
            
    else:
        i = plt_fig
        points = Sample_points[i]
        m0 = M0[i]
        m1 = M1[i]
        s0 = S0[i]
        s1 = S1[i]
        Pr_D,Pr_M,Pr_FA,Pr_CR,Points_D,Points_M,Points_FA,Points_CR = EprC.Error_prob_cal_oldV(m0,m1,s0,s1,points,True)
        visualization_2d(Points_D,Points_FA,Points_M,Points_CR)  
        title = 'Error_prob_cal Verification for data ' + str(i) 
        fig_path = './figs/error_prob_verification_2d/'
        smpl_bGDnd_plt.savefigs(title,fig_path,i,close_flg = True)
        
    return Pr_D,Pr_FA,Pr_M,Pr_CR

if __name__ == "__main__":
    
    """
    filepath = './data_storage/verification_2d/'
    n = 10
    Sample_points,M0,M1,S0,S1 = smpl_bGDnd_plt.load_plt_data(filepath) 
    points = Sample_points[n]
    m0 = M0[n]
    m1 = M1[n]
    s0 = S0[n]
    s1 = S1[n]
    p0 = multivariate_normal(mean=m1,cov=s1).pdf(points[n])
    """
    n = 151
    #Pr_D,Pr_FA,Pr_M,Pr_CR,points_D,points_FA,points_M,points_CR = EprC.Error_prob_cal(m0,m1,s0,s1,points,True)

    Epr_verification_2d(n)
    
    """
    points_D_con = np.array(points_D)
    plt.plot(points_D_con[:,0],points_D_con[:,1],'ro')
    plt.show()
    """
    #points_D_con = cnvdata.convert_array2list_nd(points_D,2)
    #plt.plot()
    #points_D_con = points_D[0]
    #points_D_conv = points_D_con + points_D[1]
    #points_D_conv = np.vstack(points_D_con,points_D[1].reshape(1,2))
    