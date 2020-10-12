# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 12:27:13 2020

@author: Zhaoliang
"""

import binary_HT_nd.sampling_binGDnd as smpl_bGDnd
from scipy.stats import norm, multivariate_normal

def Error_prob_cal_bin(mean0,mean1,Sigma0,Sigma1,points):
    count_CR = 0
    count_FA = 0
    count_M = 0
    count_D = 0
    num = len(points)
    for i in range(num):
        p0 = multivariate_normal(mean=mean0,cov=Sigma0).pdf(points[i])
        p1 = multivariate_normal(mean=mean1,cov=Sigma1).pdf(points[i])
        if i < num/2: 
            if p0 > p1:
                count_CR = count_CR + 1
            else:
                count_M = count_M + 1
        else: 
            if p0 < p1: 
                count_D = count_D + 1
            else:
                count_FA = count_FA + 1
    Pr_D = count_D/(num/2)
    Pr_M = count_M/(num/2)
    Pr_FA = count_FA/(num/2)
    Pr_CR = count_CR/(num/2)
    
    return Pr_D,Pr_FA,Pr_M,Pr_CR

if __name__ == "__main__":
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([1]).reshape(1, 1)
    Sigma1 = np.array([1]).reshape(1, 1)
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,1000)
    Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal_bin(mean0,mean1,Sigma0,Sigma1,points)
    