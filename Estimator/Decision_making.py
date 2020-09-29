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
import utility_functions.plot_figures as plotfgs
import binary_HT_1d.Gaussian_dist as Gd
import random
import matplotlib.pyplot as plt

def Decision_making(ut,ut_zt,Sigma_ut_zt):
    ut0 = ut[0]
    ut1 = ut[1]
    pdf_H0 = multivariate_normal(mean=ut0,cov=Sigma_ut_zt).pdf(ut_zt) 
    pdf_H1 = multivariate_normal(mean=ut1,cov=Sigma_ut_zt).pdf(ut_zt) 
    if pdf_H0 >= pdf_H1:
        decide_u = ut0
    else: decide_u = ut1
    return decide_u

if __name__ == '__main__':
    # randomly create some ut
    range1 = -3
    range2 = 3
    a = round(random.uniform(range1,range2));
    b = round(random.uniform(range1,range2));
    while a == b:
        a = round(random.uniform(range1,range2));
    if a > b:
        temp = a
        a = b 
        b = temp        
    ut = [a,b]
    Sigma_ut_zt = random.uniform(0,range2)
    ut_zt = random.uniform(range1,range2)
    u_D = Decision_making(ut,ut_zt,Sigma_ut_zt)
    # plot figure    
    pdf_H0 = multivariate_normal(mean=ut[0],cov=Sigma_ut_zt).pdf(ut_zt) 
    pdf_H1 = multivariate_normal(mean=ut[1],cov=Sigma_ut_zt).pdf(ut_zt)
    Gd.plot_2_Gaussian(ut[0],ut[1],round(Sigma_ut_zt,2),round(Sigma_ut_zt,2))
    plt.plot(ut_zt,pdf_H1,color='kv', linewidth=10)
    plt.plot(ut_zt,pdf_H0,color='kv', linewidth=10)

    plt.title('decision:% d'% u_D)

    
