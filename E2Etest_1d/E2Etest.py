# -*- coding: utf-8 -*-
"""
Created on Mon Aug 10 11:28:03 2020

@author: Zhaoliang
"""

import numpy as np

import Sim_KF_Pln_nd.simulation as sim1
import Sim_KF_Pln_nd.kalman_filter as kf
import Sim_KF_Pln_nd.planning as kfpln

import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import utility_functions.CompP2SHist as CompP2SHist

import binary_HT_1d.Gaussian_dist as Gd


def HT_generate_Usample_H01(A,B,H,Q,R,x0,uts,ts,sample_num):
    """
    This function is generate Uz_H0 and Uz_H1
    given input ut = 1 or 0 at the last timestep: t+1
    """
    Uz_H1 = []
    Uz_H0 = []
    timesteps = np.sum(ts)
    for i in range(sample_num):
        ground_truth_0,measurements_0,ground_truth_1,measurements_1 = sim1.generate_ut01(A,B,H,Q,R,x0,timesteps,uts)
        predict0,sigma0,estimates0 = kf.KFprocess(A,B,H,Q,R,measurements_0)
        predict1,sigma1,estimates1 = kf.KFprocess(A,B,H,Q,R,measurements_1)
        U_z_H1 = measurements_1[-1] - np.dot(H,np.dot(A,estimates1[-2]))
        U_z_H0 = np.dot(H,np.dot(A,estimates0[-2])) - measurements_0[-1]
        Uz_H1.append(U_z_H1)
        Uz_H0.append(U_z_H0)
    return Uz_H0,Uz_H1

def HT_generate_Usample(A,B,H,Q,R,x0,uts,ts,sample_num):
    """
    This function is generate arbitrary Uz
    given input ut at any time step ts
    """
    Uz_H = []
    ut_sq = sim1.generate_sequential_ut(uts,ts)
    for i in range(sample_num):
        ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq)
        predict,sigma,estimates = kf.KFprocess(A,B,H,Q,R,measurements)
        U_z_H = measurements[-1] - np.dot(H,np.dot(A,estimates[-2]))
        Uz_H.append(U_z_H)
    return Uz_H

def verify_Usample_H01():
    dx = 1
    dz = 1
    # system matrices parameters
    q = 0.2
    r = 0.1
    a = 1
    h = 1
    b = 1 
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    sample_num = 1000    
    uts = [0,1]
    ts = [100,1]
    Uz_H0,Uz_H1 = HT_generate_Usample_H01(A,B,H,Q,R,x0,uts,ts[0],sample_num)
    Uz_H1 = cnvdata.convert_array2list_1d(Uz_H1)
    Uz_H0 = cnvdata.convert_array2list_1d(Uz_H0)
    # planning USigma and Umean from KF planning
    Usigma_pln = kfpln.KF_planning_Uz(A, B, H, Q, R,ts[0])
    Umean0 = np.dot(H,B)*uts[0] 
    Umean1 = np.dot(H,B)*uts[1]        
    CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = Umean0[0],Sigma_pln = Usigma_pln[0],name='plan')
    mean_error0,var_error0 = CompP2S.numer_compare(Uz_H0)
    CompP2S.visualization_compare(Uz_H0)
    
    CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = Umean1[0],Sigma_pln = Usigma_pln[0],name='plan')
    mean_error1,var_error1 = CompP2S.numer_compare(Uz_H1)
    CompP2S.visualization_compare(Uz_H1)
    return mean_error0,var_error0,mean_error1,var_error1

def verify_Usample():
    """
    This function is to verify HT_generate_Usample function and HT_generate_Usample_H01
    see if the Uz_H can be draw as Gaussian distribution 
    with mean of BH*ut and variance of U_sigma
    """
    dx = 1
    dz = 1
    # system matrices parameters
    q = 0.2
    r = 0.1
    a = 1
    h = 1
    b = 1 
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    sample_num = 1000    
    # ==================== 
    """
    you can change the code between === to verify other hypothesis
    """
    '''
    uts = [0]
    ts = [100]
    Uz_H0 = HT_generate_Usample(A,B,H,Q,R,x0,uts,ts,sample_num)
    Uz_H0 = cnvdata.convert_array2list_1d(Uz_H0)
    samples = Uz_H0
    '''
    # =====================
    # ---------------------
    """
    If you want to verify ut = 1 then activate the code between ----
    and comment the code between ====
    """
    uts = [0,1]
    ts = [100,1]
    Uz_H1 = HT_generate_Usample(A,B,H,Q,R,x0,uts,ts,sample_num)
    Uz_H1 = cnvdata.convert_array2list_1d(Uz_H1)
    samples = Uz_H1
    # ---------------------
    # planning USigma and Umean from KF planning
    Usigma_pln = kfpln.KF_planning_Uz(A, B, H, Q, R,ts[0])
    Umean = np.dot(H,B)*uts[0] # if you want to verify Uz_H0, use this line of code
    Umean = np.dot(H,B)*uts[1] # if you want to verify Uz_H1, use this line of code
    
    CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = Umean[0],Sigma_pln = Usigma_pln[0],name='plan')
    mean_error,var_error = CompP2S.numer_compare(samples)
    CompP2S.visualization_compare(samples)
    
    return  mean_error,var_error    

def E2E_error_prob(A,B,H,Q,R,x0,uts,ts):
    """
    This function outputs theoretical error probabilities
    """
    sigma_pln0 = kfpln.KF_planning(A, B, H, Q, R,ts[0])
    sigma_pln1 = kfpln.KF_planning(A, B, H, Q, R,np.sum(ts))
    S0 = np.dot(np.dot(A,sigma_pln0),A.T)+Q
    Usigma0 = np.dot(np.dot(H,S0),H.T)+R
    S1 = np.dot(np.dot(A,sigma_pln1),A.T)+Q
    Usigma1 = np.dot(np.dot(H,S1),H.T)+R    
    mean_1 = np.dot(H,B)*uts[1]
    mean_0 = np.dot(H,B)*uts[0]

    Prob_D,Prob_FA,Prob_M,Prob_CR = Gd.error_prob(mean_0,mean_1,Usigma0,Usigma1)
    return Prob_D,Prob_FA,Prob_M,Prob_CR
    
def E2E_stats_error_prob(A,B,H,Q,R,x0,uts,ts,sam_numb,flag = 1):
    """
    This function outputs statistical error probabilities
    """
    #timesteps = np.sum(ts)
    if flag == 1:
        Uz_H0,Uz_H1 = HT_generate_Usample_H01(A,B,H,Q,R,x0,uts,ts[0],sam_numb)
    elif flag == 2:
        Uz_H1 = HT_generate_Usample(A,B,H,Q,R,x0,uts,ts,sam_numb)
        uts0 = [0]
        ts0 = [np.sum(ts)]
        Uz_H0 = HT_generate_Usample(A,B,H,Q,R,x0,uts0,ts0,sam_numb)
    
    Uz_H1 = cnvdata.convert_array2list_1d(Uz_H1)
    Uz_H0 = cnvdata.convert_array2list_1d(Uz_H0)
    samples1 = Uz_H1
    samples0 = Uz_H0
    mean1 = np.mean(Uz_H1)
    var1 = np.cov(Uz_H1)
    mean0 = np.mean(Uz_H0)
    var0 = np.cov(Uz_H0)
    
    Lambda = Gd.check_intersect_new(mean0,mean1,var0,var1)

    dlm = len(Lambda)
    count_D = 0
    count_FA = 0
    if dlm == 1:
        for i in range(sam_numb):
            if samples1[i] > Lambda[0]:
                count_D = count_D + 1
            if samples0[i] > Lambda[0]:
                count_FA = count_FA + 1
        Prob_D = count_D / sam_numb
        Prob_FA = count_FA / sam_numb
        Prob_M = 1 - Prob_D
        Prob_CR = 1 - Prob_FA
    elif dlm!=1 and var0 > var1:    
        for i in range(sam_numb):
            if samples1[i] > Lambda[0] and samples1[i] < Lambda[1]:
                count_D = count_D + 1
            if samples0[i] > Lambda[0] and samples0[i] < Lambda[1]: 
                count_FA = count_FA + 1
        Prob_D = count_D / sam_numb
        Prob_FA = count_FA / sam_numb
        Prob_M = 1 - Prob_D
        Prob_CR = 1 - Prob_FA
    elif dlm!=1 and var1 > var0: 
        count_M = 0
        count_CR = 0
        for i in range(sam_numb):
            if samples1[i] > Lambda[0] and samples1[i] < Lambda[1]:
                count_M = count_M + 1
            if samples0[i] > Lambda[0] and samples0[i] < Lambda[1]: 
                count_CR = count_CR + 1
                
        Prob_M = count_M / sam_numb
        Prob_CR = count_CR / sam_numb
        Prob_D = 1 - Prob_M
        Prob_FA = 1 - Prob_CR
    return Prob_D,Prob_FA,Prob_M,Prob_CR    

def E2E_comp(A,B,H,Q,R,x0,uts,ts,sample_num):
    """
    This function is a numeric comparison between
    theoretical error probabilities (E2E_error_prob)and 
    statistical error probabilities (E2E_stats_error_prob)
    """
    Prob_D_sta,Prob_FA_sta,Prob_M_sta,Prob_CR_sta = E2E_stats_error_prob(A,B,H,Q,R,x0,uts,ts,sample_num)
    Prob_D,Prob_FA,Prob_M,Prob_CR  = E2E_error_prob(A,B,H,Q,R,x0,uts,ts)
    error_D = Prob_D_sta - Prob_D
    error_FA = Prob_FA_sta - Prob_FA
    error_M = Prob_M_sta - Prob_M
    error_CR = Prob_CR_sta - Prob_CR
    return error_D,error_FA,error_M,error_CR

def E2E_Comptest():
    """
    This function is to search the parameters space 
    You can change parameters using while loop
    for each parameters, we will test 1000 sample trials
    """
    dx = 1
    dz = 1
    # system matrices parameters
    q = 0.2
    r = 0.1
    a = 1
    h = 1
    b = 1
    x0 = np.array([[0]]).reshape(dx, 1)
    sample_num = 1000
    
    uts = [0,1]
    ts = [100,1]
    Error_D = []
    Error_FA = []
    Error_M = []
    Error_CR = []
    while r<=2:
        A = np.array([a]).reshape(dx, dx)
        H = np.array([h]).reshape(dz, dx)
        B = np.array([b]).reshape(dx, 1)
        Q = np.array([q * q]).reshape(dx, dx)
        R = np.array([r * r]).reshape(dz, dz)
        error_D,error_FA,error_M,error_CR = E2E_comp(A,B,H,Q,R,x0,uts,ts,sample_num)
        """
        print("new case:")
        print(error_D)
        print(error_FA)
        print(error_M)
        print(error_CR)
        """
        Error_D.append(error_D)
        Error_FA.append(error_FA)
        Error_M.append(error_M)
        Error_CR.append(error_CR)
        r = r + 0.2
    print(np.mean(Error_D))
    print(np.mean(Error_FA))
    print(np.mean(Error_M))
    print(np.mean(Error_CR))
    return Error_D,Error_FA,Error_M,Error_CR

def example_E2E_error_prob():
    dx = 1
    dz = 1
    # system matrices parameters
    q = 0.2
    r = 0.1
    a = 1
    h = 1
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q * q]).reshape(dx, dx)
    R = np.array([r * r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    uts = [0,1]
    ts = [100,1]
    
    Prob_D,Prob_FA,Prob_M,Prob_CR = E2E_error_prob(A,B,H,Q,R,x0,uts,ts)
    return Prob_D,Prob_FA,Prob_M,Prob_CR
        
if __name__ == '__main__':
    # one example to show how to calcualte E2E error probabilities
    Prob_D,Prob_FA,Prob_M,Prob_CR = example_E2E_error_prob()
    """
    Uncomment next two lines of code to varify HT_generate_Usample_H01 
    and HT_generate_Usample
    """
    #mean_error0,var_error0,mean_error1,var_error1 = verify_Usample_H01()
    #mean_error,var_error = verify_Usample()
    """
    Uncomment next line of code to run 
    comprehensive End2End 1d binary hypothesis testing
    This comprehensive test will be running very slow, please be patient
    """
    #Error_D,Error_FA,Error_M,Error_CR = E2E_Comptest()
