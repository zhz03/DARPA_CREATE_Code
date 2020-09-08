# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 00:08:35 2020

@author: LENOVO
"""
import numpy as np
from scipy.stats import multivariate_normal
import Estimator.KF_estimator as KF_est
import Estimator.Bayesian_analysis as BA
import Simulation.Generate_seq_u as Gsequ
import Simulation.Simulator as Simu
import utility_functions.convert_data as cnvdata


def Decision_making(ut,ut_zt,Sigma_ut_zt):
    ut0 = ut[0]
    ut1 = ut[1]
    pdf_H0 = multivariate_normal(mean=ut0,cov=Sigma_ut_zt).pdf(ut_zt) 
    pdf_H1 = multivariate_normal(mean=ut1,cov=Sigma_ut_zt).pdf(ut_zt) 
    if pdf_H0 > pdf_H1:
        decide_u = 0
    else: decide_u = 1
    return decide_u

if __name__ == '__main__':

    ut_zt = 0.8
    Sigma_ut_zt = 0.5    
    ut = [0,1]
    u_D = Decision_making(ut,ut_zt,Sigma_ut_zt)