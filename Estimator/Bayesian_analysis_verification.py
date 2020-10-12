# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 00:40:28 2020

@author: Zhaoliang
"""

import Estimator.Bayesian_analysis as BA
import Estimator.KF_estimator as KF_est
import Simulations.Generate_seq_u as Gsequ
import Simulations.Simulator as Simu
import utility_functions.convert_data as cnvdata
import utility_functions.CompP2SHist as CompP2SHist
import matplotlib.pyplot as plt
import E2Etest.SM_generator_1d as SMGen1d
import E2Etest.System_setup_generator as SSGen

def verification(num):
    
    dx = 1
    Arange = [1,1]
    Brange = [1,1]
    Hrange = [0,2]
    Qrange = [0,2]    
    Rrange = [0,2]
    System_models = SMGen1d.SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange)
    #As,Hs,Bs,Qs,Rs,
    T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
    SM_num = len(System_models[0])
    for i in range(SM_num):
        A = System_models[0][i]
        B = System_models[1][i]
        H = System_models[2][i]
        Q = System_models[3][i]
        R = System_models[4][i]
        SM = [A,B,H,Q,R]
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
        fig_name = './figs/Bayesian_analysis_figs/' + str(i) + '_UtztHist.jpg'
        plt.savefig(fig_name)
        plt.close()
        
if __name__ == '__main__':
    verification(20)