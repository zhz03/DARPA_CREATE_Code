# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:02:38 2020

@author: Zhaoliang
"""

import numpy as np
import Estimator.KF_estimator as KF_est
import Estimator.Bayesian_analysis as BA
import Estimator.Decision_making as DM
import Simulations.Generate_seq_u as Gsequ
import Simulations.Simulator as Simu
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import E2Etest.SM_generator_1d as SMGen1d
import E2Etest.System_setup_generator as SSGen
import matplotlib.pyplot as plt
import utility_functions.CompP2SHist as CompP2SHist
from scipy.stats import multivariate_normal



def estimator(SM,z,ut):
    Sigma,estimates = KF_est.KF_estimator(SM,z)
    ut_zt,mean_ut_zt,Sigma_ut_zt = BA.Bayesian_analysis(SM,ut,Sigma,estimates,z)
    u_D = DM.Decision_making(ut,ut_zt,mean_ut_zt,Sigma_ut_zt)
    return u_D

def verification(num):

    dx = 1
    Arange = [1,1]
    Brange = [0,2]
    Hrange = [0,2]
    Qrange = [0,2]    
    Rrange = [0,2]
    System_models = SMGen1d.SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange)
    #As,Hs,Bs,Qs,Rs,
    T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
    SM_num = len(System_models[0])
    T = 1000
    ts[0] = T
    for i in range(SM_num):
        A = System_models[0][i]
        B = System_models[1][i]
        H = System_models[2][i]
        Q = System_models[3][i]
        R = System_models[4][i]
        SM = [A,B,H,Q,R]
        u = Gsequ.generate_sequential_ut(uts,ts)
        y,z = Simu.Simulator(SM,x0,u)
        #==============estimator===============================
        Sigma,estimates = KF_est.KF_estimator(SM,z)
        ut_zt,mean_ut_zt,Sigma_ut_zt = BA.Bayesian_analysis(SM,ut,Sigma,estimates,z)
        u_D = DM.Decision_making(ut,ut_zt,mean_ut_zt,Sigma_ut_zt)
        #======================kf_estimator_verification===============================

        ground_truth = cnvdata.convert_array2list_nd(y,dx)
        measurements = cnvdata.convert_array2list_nd(z,dx) 
        estimates = cnvdata.convert_array2list_nd(estimates,dx)
        errors = [estimates[0][i] - ground_truth[0][i] for i in range(len(estimates[0]))]
        plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name = './figs/estimator_figs/' + str(i) + 'kf_estimator.jpg'
        plt.savefig(fig_name)
        plt.close()

        CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = 0, Sigma_pln = Sigma[-1][0][0])
        CompP2S.visualization_compare(errors,1)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name1 = './figs/estimator_figs/' + str(i) + '_error.jpg'
        plt.savefig(fig_name1)
        plt.close()
        # ========================verify bayesian analysis in the estimator 
        """
        Ut_zt = []
        samp_num = 100
        for j in range(samp_num):
            u = Gsequ.generate_sequential_ut(uts,ts)
            y,z = Simu.Simulator(SM,x0,u)
            Sigma,estimates = KF_est.KF_estimator(SM,z)
            ut_zt,mean_ut_zt,Sigma_ut_zt = BA.Bayesian_analysis(SM,ut,Sigma,estimates,z)
            Ut_zt.append(ut_zt)
        Ut_zt = cnvdata.convert_array2list_nd(Ut_zt,dx)
        #mean_pln = np.dot(H,B)*uts[1]
        CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = mean_ut_zt[1][0][0], Sigma_pln = Sigma_ut_zt[0][0][0])
        CompP2S.visualization_compare(Ut_zt[0],1)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name = './figs/estimator_figs/' + str(i) + '_UtztHist.jpg'
        plt.savefig(fig_name)
        plt.close()
        """
        #================ Decision making verification   
        pdf_H0 = multivariate_normal(mean=mean_ut_zt[0],cov=Sigma_ut_zt[0]).pdf(ut_zt) 
        pdf_H1 = multivariate_normal(mean=mean_ut_zt[1],cov=Sigma_ut_zt[1]).pdf(ut_zt)
        plotfgs.plot_2_Gaussian_withpoints(mean_ut_zt[0][0],mean_ut_zt[1][0],round(Sigma_ut_zt[0][0][0],2),round(Sigma_ut_zt[1][0][0],2),ut_zt[0][0],pdf_H1,ut_zt[0][0],pdf_H0)
        plt.title('decision:% d'% u_D)
        fig_name = './figs/estimator_figs/' + str(i) + 'DecisionMaking' +'.jpg'
        plt.savefig(fig_name)
        plt.close()
if __name__ == '__main__':
    num = 100
    KF_est.verification(num)
    BA.verification(num)
    DM.verification(num)
    verification(num)
    
    # system matrices parameters
    """
    q = 0.3
    r = 0.2
    a = 1
    h = .1
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    uts = [0,1]
    ts = [100,1]
    ut = [0,1]
    SM = [A,B,H,Q,R]
    """
    """
    dx = 1
    dz = 1
    q = .1254
    r = 1.6798
    r = .1254
    a = 1.4816
    #a = 1.0
    h = 1.5927
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q]).reshape(dx, dx)
    R = np.array([r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    
    uts = [0,1]
    ts = [100,1]
    ut = [0,1] 
    trials = 1000
    SM = [A,B,H,Q,R]
    
    u = Gsequ.generate_sequential_ut(uts,ts)
    y,z = Simu.Simulator(SM,x0,u)
    Sigma,estimates = KF_est.KF_estimator(SM,z)
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx) 
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    Sigma = cnvdata.convert_array2list_nd(Sigma,dx)
    
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
    """
    #u_D = estimator(SM,z,ut)
    