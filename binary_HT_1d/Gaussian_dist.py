# -*- coding: utf-8 -*-
"""
Created on Thu Jun 25 14:18:05 2020

@author: Zhaoliang
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

def gaussian(x, mu, delta):
    exp = np.exp(- np.power(x - mu, 2) / (2 * np.power(delta, 2)))
    c = 1 / (delta * np.sqrt(2 * np.pi))
    return c * exp

def plotGaussian(x,mean,var,Color,Label):
    y = gaussian(x, mean, np.sqrt(var))
    plt.plot(x, y,Color,label=Label)
    plt.legend()

def plot_2_Gaussian(mean0,mean1,var0,var1):
    x = generate_x(mean0,mean1,var0,var1)    
    plotGaussian(x,mean0,var0,'r.','mean={},var={}'.format(mean0,var0))
    plotGaussian(x,mean1,var1,'b.','mean={},var={}'.format(mean1,var1))  
    
def pre_check_interect(x,y1,y2):
    dnx = len(x)
    Lambda = []
    for i in range(dnx):
        if y1[i] > 0.01 and y2[i] > 0.01 and np.abs(y1[i] - y2[i]) < 0.001:
            Lambda.append(x[i])
    return Lambda

def filter_lambda(Lambda):
    flambda = []
    d = len(Lambda)
    if d==1:
        flambda = Lambda
    elif d!=1:
        for i in range(d-1):
            sumlambda = []
            if np.abs(Lambda[i]-Lambda[i+1]) > 0.02:
                flambda.append(Lambda[i])
                if i+1 == d-1:
                    flambda.append(Lambda[i+1])
            else:
                sumlambda.append(Lambda[i])
                #print(i)
                #print(Lambda[i])
        if len(sumlambda)!=0:
            flambda.append(np.mean(sumlambda))
        flambda.sort()
    return flambda        

def generate_x(mean0,mean1,var0,var1):
    std0 = np.sqrt(var0)
    std1 = np.sqrt(var1)
    if std0 >= std1:
        std = std0
    else:
        std = std1
    if mean0 < mean1:
        range_s = mean0 - 3 * std
        range_e = mean1 + 3 * std
    if mean0 > mean1:
        range_s = mean1 - 3 * std
        range_e = mean0 + 3 * std
    space_se = (range_e-range_s)/0.001
    x = np.linspace(range_s,range_e,int(space_se))
    return x

def check_intersect_new(mean0,mean1,var0,var1):
    x = generate_x(mean0,mean1,var0,var1)
    y1 = gaussian(x, mean0, np.sqrt(var0))
    y2 = gaussian(x, mean1, np.sqrt(var1))
    
    Lambda = pre_check_interect(x,y1,y2)
    
    if len(Lambda)==0:
        Lambda.append((mean1+mean0)/2)
    else:
        Lambda = filter_lambda(Lambda)
    return Lambda
    
def cdfd(start,end,mean,var):
    cdf1 = multivariate_normal(mean=mean,cov=var).cdf(start)  
    cdf2 = multivariate_normal(mean=mean,cov=var).cdf(end)
    return cdf2 -cdf1
    
def error_prob(mean0,mean1,var0,var1):
    """ This function is to calculate four possibilities:
        1.  𝐻_1 is true, decide 𝐻_1 : Prob_D
        2. 𝐻_0 is true, decide 𝐻_1 : Prob_FA
        3. 𝐻_1 is true, decide 𝐻_0 : Prob_M = 1 - Prob_D
        4. 𝐻_0 is true, decide 𝐻_0 : Prob_CR
    Given two Two hypotheses:
        H_1： u_t= 0 -> mean0 (no events happen case)
        H_2: u_t= 1 -> mean1 (events happen case)
    """

    Lambda = check_intersect_new(mean0,mean1,var0,var1)
    
    INF = 100
    dlm = len(Lambda)
    if dlm == 1:
        Prob_D = cdfd(Lambda[0],INF,mean1,var1)
        Prob_FA = cdfd(Lambda[0],INF,mean0,var0)
        Prob_M = 1 - Prob_D
        Prob_CR = 1 - Prob_FA
    elif dlm!= 1 and var0 > var1:
        Prob_D = cdfd(Lambda[0],Lambda[1],mean1,var1)
        Prob_FA = cdfd(Lambda[0],Lambda[1],mean0,var0)
        Prob_M = 1 - Prob_D
        Prob_CR = 1 - Prob_FA
    elif dlm != 1 and var1 > var0:
        Prob_M = cdfd(Lambda[0],Lambda[1],mean1,var1)
        Prob_CR = cdfd(Lambda[0],Lambda[1],mean0,var0)
        Prob_D = 1 - Prob_M
        Prob_FA = 1 - Prob_CR

    
    Pr_D,Pr_FA,Pr_M,Pr_CR = Prob_D,Prob_FA,Prob_M,Prob_CR
    return Pr_D,Pr_FA,Pr_M,Pr_CR 

def example1():
    """
    one example to show how to calculate error probabilities
    using mean0,mean1,var0,var1
    you can change mean0,mean1,var0,var1 as you want
    """
    mean0 = 0
    var0 = 0.1
    mean1 = 1
    var1 = 1
    Prob_D,Prob_FA,Prob_M,Prob_CR =  error_prob(mean0,mean1,var0,var1)
    return Prob_D,Prob_FA,Prob_M,Prob_CR
            
if __name__ == '__main__':
    #Prob_D,Prob_FA,Prob_M,Prob_CR = example1()
    mean0 = 0
    var0 = 0.1
    mean1 = 1
    var1 = 1
    
    plot_2_Gaussian(mean0,mean1,var0,var1)
    x = generate_x(mean0,mean1,var0,var1)
    y1 = gaussian(x, mean0, np.sqrt(var0))
    y2 = gaussian(x, mean1, np.sqrt(var1))
    
    Lambda = pre_check_interect(x,y1,y2)
    
    Lambda1 = filter_lambda(Lambda)
    

    
    #Prob_D,Prob_FA,Prob_M,Prob_CR =  error_prob(mean0,mean1,var0,var1)
    

    
    