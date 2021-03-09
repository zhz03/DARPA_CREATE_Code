# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 00:08:35 2020

@author: LENOVO
"""
import numpy as np
from scipy.stats import multivariate_normal


def Decision_making(ut,ut_zt,mean_ut_zt,Sigma_ut_zt):
    Sigma_ut_zt0 = Sigma_ut_zt[0]
    Sigma_ut_zt1 = Sigma_ut_zt[1]
    mean0 = mean_ut_zt[0]
    mean1 = mean_ut_zt[1]
    ut0 = ut[0]
    ut1 = ut[1]
    print("ut is ", ut)
    pdf_H0 = multivariate_normal(mean=mean0,cov=Sigma_ut_zt0).pdf(ut_zt) 
    pdf_H1 = multivariate_normal(mean=mean1,cov=Sigma_ut_zt1).pdf(ut_zt) 
    if pdf_H0 >= pdf_H1:
        decide_u = ut0
    else: decide_u = ut1
    return decide_u

if __name__ == '__main__':
    pass


    
