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
from scipy.stats import norm, multivariate_normal

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
    mean0 = np.array([0,0])
    mean1 = np.array([1,1])
    Sigma0 = np.array([[2,-1],[-1,2]]).reshape(2, 2)
    Sigma1 = np.array([[3,0.1],[0.1,3]]).reshape(2, 2)
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,1000)

    #p0 = multivariate_normal(mean=mean0,cov=Sigma0).pdf(points[0])
    p1 = multivariate_normal(mean=mean1,cov=Sigma1).pdf(points[0])  
    
    filepath = './data_storage/verification_2d/'
    Sample_points,M0,M1,S0,S1 = smpl_bGDnd_plt.load_plt_data(filepath) 
    points1 = Sample_points[1]
    m0 = M0[1]
    m1 = M1[1]
    s0 = S0[1]
    s1 = S1[1]
    p0 = multivariate_normal(mean=m0,cov=s0).pdf(points[1])
    
  
    #Pr_D,Pr_FA,Pr_M,Pr_CR,Points_D,Points_FA,Points_M,Points_CR = EprC.Error_prob_cal(m0,m1,s0,s1,points,True)
    #plt.plot()
    