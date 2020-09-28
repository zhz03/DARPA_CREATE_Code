# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 17:03:24 2020

@author: Zhaoliang
"""
import numpy as np
from numpy.random import randn
import Gaussian_dist as Gd
import utility_functions.CompP2SHist as CompP2SHist

def generate_gaussian_samples(mean,var,samn):
    """
    For random samples from N(\mu, \sigma^2)
    """
    samples =[]
    for i in range(samn):
        sam1 = np.sqrt(var) * randn() + mean
        samples.append(sam1)
    return samples

def verify_generated_data(mean,var,samples):
    """
    This function is to verify the generate_gaussian_samples
    by comparing statistical mean and variance with
    planning mean and variance
    """ 
    CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = mean,Sigma_pln = var,name='plan')
    mean_error,var_error = CompP2S.numer_compare(samples)
    CompP2S.visualization_compare(samples)
    return mean_error,var_error

def stats_error_prob(mean0,mean1,var0,var1,sam_numb):
    """
    This function is to calculate error probabilities 
    using statistial data 
    """
    samples0 = generate_gaussian_samples(mean0,var0,sam_numb)
    samples1 = generate_gaussian_samples(mean1,var1,sam_numb)
    Lambda = Gd.check_intersect_new(mean0,mean1,var0,var1)
    
    dlm = len(Lambda)
    count_D = 0
    count_FA = 0
    if dlm == 1 and (var0 - var1)< 0.0000001:
        for i in range(sam_numb):
            if samples1[i] > Lambda[0]:
                count_D = count_D + 1
            if samples0[i] > Lambda[0]:
                count_FA = count_FA + 1
        Prob_D = count_D / sam_numb
        Prob_FA = count_FA / sam_numb
        Prob_M = 1 - Prob_D
        Prob_CR = 1 - Prob_FA
    elif dlm != 1 and var0 > var1:    
        for i in range(sam_numb):
            if samples1[i] > Lambda[0] and samples1[i] < Lambda[1]:
                count_D = count_D + 1
            if samples0[i] > Lambda[0] and samples0[i] < Lambda[1]: 
                count_FA = count_FA + 1
        Prob_D = count_D / sam_numb
        Prob_FA = count_FA / sam_numb
        Prob_M = 1 - Prob_D
        Prob_CR = 1 - Prob_FA
    elif dlm != 1 and var1 > var0: 
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

"""
The following are example code
"""

def example1():
    """
    This example is to show how to verify generate_gaussian_samples function
    You can change mean0,mean1,var0,var1, sam_numb as you want
    """
    mean0 = 0
    var0 = 0.2
    mean1 = 5
    var1 = 10
    sam_numb = 1000
    samples0 = generate_gaussian_samples(mean0,var0,sam_numb)      
    mean_error0,var_error0 = verify_generated_data(mean0,var0,samples0)
    samples1 = generate_gaussian_samples(mean1,var1,sam_numb)      
    mean_error1,var_error1 = verify_generated_data(mean1,var1,samples1)

def example2():
    """
    This example is to show can to calculate error probabilities
    using only mean0,mean1,var0,var1
    You can change mean0,mean1,var0,var1, sam_numb as you want
    """        
    mean0 = 0
    var0 = 0.2
    mean1 = 1
    var1 = 10
    sam_numb = 1000
    Prob_D,Prob_FA,Prob_M,Prob_CR = stats_error_prob(mean0,mean1,var0,var1,sam_numb)
if __name__ == '__main__':
    example1() 
    

    

    

