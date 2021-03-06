# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 23:37:49 2020

@author: Zhaoliang
"""

import numpy as np

def Bayesian_analysis(SM,ut,Sigma,estimates,measurements):
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    ut_zt = measurements[-1] - np.dot(H,np.dot(A,estimates[-2]))
    Sigma_t = Sigma[-1]
    S = np.dot(np.dot(A,Sigma_t),A.T)+Q
    Sigma_ut_zt01 = np.dot(np.dot(H,S),H.T)+R
    
    mean_ut_zt = [np.dot(H,B)*ut[0],np.dot(H,B)*ut[1]]
    Sigma_ut_zt = [Sigma_ut_zt01,Sigma_ut_zt01]
    return ut_zt,mean_ut_zt,Sigma_ut_zt

    
if __name__ == '__main__':
    verification(20)
    
    """
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    """
    
    