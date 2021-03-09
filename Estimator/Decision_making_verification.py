# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 10:31:31 2020

@author: Zhaoliang
"""

import numpy as np
from scipy.stats import multivariate_normal
import Estimator.KF_estimator as KF_est
import Estimator.Bayesian_analysis as BA
import Simulations.Generate_seq_u as Gsequ
import Simulations.Simulator as Simu
import utility_functions.convert_data as cnvdata
import utility_functions.plot_figures as plotfgs
import binary_HT_1d.Gaussian_dist as Gd
import random
import matplotlib.pyplot as plt
import Estimator.Decision_making as DM

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
        mean_ut_zt = ut
        Sigma_ut_zt0 = random.uniform(0,range2)
        Sigma_ut_zt1 = random.uniform(0,range2)
        Sigma_ut_zt = [Sigma_ut_zt0,Sigma_ut_zt1]
        ut_zt = random.uniform(ut[0]-3*np.sqrt(Sigma_ut_zt[0]),ut[1]+3*np.sqrt(Sigma_ut_zt[1]))
        u_D = DM.Decision_making(ut,ut_zt,mean_ut_zt,Sigma_ut_zt)
        # plot figure    
        pdf_H0 = multivariate_normal(mean=ut[0],cov=Sigma_ut_zt[0]).pdf(ut_zt) 
        pdf_H1 = multivariate_normal(mean=ut[1],cov=Sigma_ut_zt[1]).pdf(ut_zt)
        plotfgs.plot_2_Gaussian_withpoints(ut[0],ut[1],round(Sigma_ut_zt[0],2),round(Sigma_ut_zt[1],2),ut_zt,pdf_H1,ut_zt,pdf_H0)
        plt.title('decision:% d'% u_D)
        fig_name = './figs/Decision_making_figs/' + str(i) + '.jpg'
        plt.savefig(fig_name)
        plt.close()
if __name__ == '__main__':
    verification(2)

