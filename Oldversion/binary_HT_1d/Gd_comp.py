# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 17:48:51 2020

@author: Zhaoliang
"""

import numpy as np
import Gaussian_dist as Gd
import Gd_stat as Gd_statis
import utility_functions.convert_data as cnvdata

def mean_cov(sample):
    sam_mean = np.mean(sample)
    sam_cov = np.cov(sample)
    return sam_mean, sam_cov 
    
def error_comp(mean0,mean1,var0,var1,sam_numb):
    Prob_sta = Gd_statis.stats_error_prob(mean0,mean1,var0,var1,sam_numb)
    Prob_D_sta = Prob_sta[0]
    Prob_FA_sta = Prob_sta[1]
    Prob_M_sta = Prob_sta[2]
    Prob_CR_sta = Prob_sta[3]
    
    # If you want to see the Plot, change Plotflag = True
    Prob_D,Prob_FA,Prob_M,Prob_CR =  Gd.error_prob(mean0,mean1,var0,var1,Plotflag = False)
    
    error_D = Prob_D_sta - Prob_D
    error_FA = Prob_FA_sta - Prob_FA
    error_M = Prob_M_sta - Prob_M
    error_CR = Prob_CR_sta - Prob_CR
    return error_D,error_FA,error_M,error_CR    

def error_comp_example():
    """
    This is one example to show how to
    calculate the error between 
    statistical Error prob and theoretical Error prob
    """
    mean0 = 0
    var0 = 10
    mean1 = 1
    var1 = 0.4
    sam_numb = 1000
    
    Prob_sta = Gd_statis.stats_error_prob(mean0,mean1,var0,var1,sam_numb)
    Prob_D_sta = Prob_sta[0]
    Prob_FA_sta = Prob_sta[1]
    Prob_M_sta = Prob_sta[2]
    Prob_CR_sta = Prob_sta[3]
    
    Prob_D,Prob_FA,Prob_M,Prob_CR =  Gd.error_prob(mean0,mean1,var0,var1)
    
    error_D = Prob_D_sta - Prob_D
    error_FA = Prob_FA_sta - Prob_FA
    error_M = Prob_M_sta - Prob_M
    error_CR = Prob_CR_sta - Prob_CR
    return error_D,error_FA,error_M,error_CR    

def comprehensive_test1():
    """
    This function is to test the error between 
    statistical Error prob and theoretical Error prob
    with different mean with same var
    """
    mean0 = 0
    mean1 = 1 
    Mean = [-10,-2,-1,1,1.5,2,5,10]
    var0 = 0.5
    var1 = 0.5
    sam_numb = 1000
    Errors_D = []
    Errors_FA = []
    Errors_M = []
    Errors_CR = []
    for i in range(len(Mean)):
        mean1 = Mean[i] #mean1 can be change into mean0
        error_D,error_FA,error_M,error_CR = error_comp(mean0,mean1,var0,var1,sam_numb)
        Errors_D.append(error_D)
        Errors_FA.append(error_FA)
        Errors_M.append(error_M)
        Errors_CR.append(error_CR)
    D_Emean,D_Ecov = mean_cov(Errors_D)
    FA_Emean,FA_Ecov = mean_cov(Errors_FA)
    M_Emean,M_Ecov = mean_cov(Errors_M)
    CR_Emean,CR_Ecov = mean_cov(Errors_CR)
    return D_Emean,D_Ecov,FA_Emean,FA_Ecov,M_Emean,M_Ecov,CR_Emean,CR_Ecov

def comprehensive_test2():
    """
    This function is to test the error between 
    statistical Error prob and theoretical Error prob
    with different var with same mean
    """
    mean0 = 0
    mean1 = 1 
    Var = [0.1,0.5,0.8,1.0,2,4,6,10] # this var has to be possitive
    var0 = 0.5
    var1 = 0.5
    sam_numb = 1000
    Errors_D = []
    Errors_FA = []
    Errors_M = []
    Errors_CR = []
    for i in range(len(Var)):
        var1 = Var[i] # var1 can be change into var0
        error_D,error_FA,error_M,error_CR = error_comp(mean0,mean1,var0,var1,sam_numb)
        Errors_D.append(error_D)
        Errors_FA.append(error_FA)
        Errors_M.append(error_M)
        Errors_CR.append(error_CR)
    D_Emean,D_Ecov = mean_cov(Errors_D)
    FA_Emean,FA_Ecov = mean_cov(Errors_FA)
    M_Emean,M_Ecov = mean_cov(Errors_M)
    CR_Emean,CR_Ecov = mean_cov(Errors_CR)
    return D_Emean,D_Ecov,FA_Emean,FA_Ecov,M_Emean,M_Ecov,CR_Emean,CR_Ecov
        
if __name__ == '__main__':
    error_D,error_FA,error_M,error_CR  = error_comp_example()
    # D_Emean,D_Ecov,FA_Emean,FA_Ecov,M_Emean,M_Ecov,CR_Emean,CR_Ecov = comprehensive_test2()
    

    