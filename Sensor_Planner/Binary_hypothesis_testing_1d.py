# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 21:55:57 2020

@author: Zhaoliang
"""

import numpy as np
import binary_HT_1d.Gaussian_dist as Gd

def Bin_Hyp_test_1d(Sigma_ut,ut):
    Sigma_ut0 = Sigma_ut[0]
    Sigma_ut1 = Sigma_ut[1]
    ut0 = ut[0]
    ut1 = ut[1]

    Pr_D,Pr_FA,Pr_M,Pr_CR = Gd.error_prob(ut0,ut1,Sigma_ut0,Sigma_ut1)
    return Pr_D,Pr_FA,Pr_M,Pr_CR

if __name__ == "__main__":
    Sigma_ut = [8,8]
    ut = [0,1]
    Prob_D,Prob_FA,Prob_M,Prob_CR = Bin_Hyp_test_1d(Sigma_ut,ut)