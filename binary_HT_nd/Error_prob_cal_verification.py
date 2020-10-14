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
#from scipy.stats import norm, multivariate_normal

#def visualization_2d():
    

def Epr_verification_2d():
    filepath = './data_storage/verification_2d/'
    Sample_points,M0,M1,S0,S1 = smpl_bGDnd_plt.load_plt_data(filepath)
    
    trial_num = len(M0)
    for i in range(trial_num):
        points = Sample_points[i]
        m0 = M0[i]
        m1 = M1[i]
        s0 = S0[i]
        s1 = S1[i]
        Pr_D,Pr_FA,Pr_M,Pr_CR,points_D,points_FA,points_M,points_CR = EprC.Error_prob_cal(m0,m1,s0,s1,points,True)
        
        
        
    return Pr_D,Pr_FA,Pr_M,Pr_CR

if __name__ == "__main__":
    
    filepath = './data_storage/verification_2d/'
    Sample_points,M0,M1,S0,S1 = smpl_bGDnd_plt.load_plt_data(filepath) 
    points = Sample_points[1]
    m0 = M0[1]
    m1 = M1[1]
    s0 = S0[1]
    s1 = S1[1]
    #p0 = multivariate_normal(mean=m0,cov=s0).pdf(points[1])
    
    Pr_D,Pr_FA,Pr_M,Pr_CR,points_D,points_FA,points_M,points_CR = EprC.Error_prob_cal(m0,m1,s0,s1,points,True)
    points_D_con = np.array(points_D)
    plt.plot(points_D_con[:,0],points_D_con[:,1],'ro')
    plt.show()
    
    #points_D_con = cnvdata.convert_array2list_nd(points_D,2)
    #plt.plot()
    #points_D_con = points_D[0]
    #points_D_conv = points_D_con + points_D[1]
    #points_D_conv = np.vstack(points_D_con,points_D[1].reshape(1,2))
    