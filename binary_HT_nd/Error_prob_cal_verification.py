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

def Epr_verification_1d():
    filepath = './data_storage/verification_1d/'
    Sample_points,M0,M1,S0,S1 = smpl_bGDnd_plt.load_plt_data(filepath)
    
    #for 
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([1]).reshape(1, 1)
    Sigma1 = np.array([1]).reshape(1, 1)
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,1000)
    Pr_D,Pr_FA,Pr_M,Pr_CR = EprC.Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points)
    return Pr_D,Pr_FA,Pr_M,Pr_CR

if __name__ == "__main__":
    filepath = './data_storage/verification_1d/'
    Sample_points,M0,M1,S0,S1 = smpl_bGDnd_plt.load_plt_data(filepath) 
    points = Sample_points[0]
    m0 = M0[0]
    m1 = M1[0]
    s0 = S0[0]
    s1 = S1[0]
