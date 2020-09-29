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

def verification(num):

    for i in range(num):
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
        ut_zt = random.uniform(ut[0]-3*np.sqrt(Sigma_ut_zt),ut[1]+3*np.sqrt(Sigma_ut_zt))
        u_D = Decision_making(ut,ut_zt,Sigma_ut_zt)
        # plot figure    
        pdf_H0 = multivariate_normal(mean=ut[0],cov=Sigma_ut_zt).pdf(ut_zt) 
        pdf_H1 = multivariate_normal(mean=ut[1],cov=Sigma_ut_zt).pdf(ut_zt)
        plotfgs.plot_2_Gaussian_withpoints(ut[0],ut[1],round(Sigma_ut_zt,2),round(Sigma_ut_zt,2),ut_zt,pdf_H1,ut_zt,pdf_H0)
        plt.title('decision:% d'% u_D)
        fig_name = './figs/Decision_making_figs/' + str(i) + '.jpg'
        plt.savefig(fig_name)
        plt.close()
if __name__ == '__main__':
    verification(20)


    
