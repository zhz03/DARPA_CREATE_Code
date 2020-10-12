# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 12:46:00 2020

@author: Zhaoliang
"""

import numpy as np
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd
import binary_HT_nd.Error_prob_cal_bin as Epr_b

def bin_HT_nd(mean0,mean1,Sigma0,Sigma1,Num_sam):
     points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,Num_sam)
     Pr_D,Pr_FA,Pr_M,Pr_CR = Epr_b.Error_prob_cal_bin(mean0,mean1,Sigma0,Sigma1,points)
     
     return Pr_D,Pr_FA,Pr_M,Pr_CR
 
    
if __name__ == "__main__":
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([1]).reshape(1, 1)
    Sigma1 = np.array([1]).reshape(1, 1)
    Pr_D,Pr_FA,Pr_M,Pr_CR = bin_HT_nd(mean0,mean1,Sigma0,Sigma1,2000)