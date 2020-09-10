# -*- coding: utf-8 -*-
"""
Created on Fri Jul  3 11:26:10 2020

@author: Zhaoliang
"""

import numpy as np
import matplotlib.pyplot as plt
import random as rnd
from scipy import linalg
import binary_HT_1d.Gaussian_dist as Gd

class Compare_pln2statis_hist(object):
    
    def __init__(self, mean_pln = None, Sigma_pln = None,name = 'plan'):
        if(mean_pln is None or Sigma_pln is None):
            raise ValueError("Set proper statistics.")
        
        self.mean_Pln =  mean_pln
        self.Sigma_Pln = Sigma_pln
        self.name = name
    
    def calculate_stat(self,data):
        mean_stat = np.mean(data)
        var_stat = np.var(data)
        return mean_stat,var_stat
    
    def numer_compare(self,data):
        mean_stat,var_stat = self.calculate_stat(data)
        mean_error = np.abs(mean_stat- self.mean_Pln)
        var_error = np.abs(var_stat - self.Sigma_Pln)
        return mean_error,var_error
    
    def visualization_compare(self,data):
        mean_stat,var_stat = self.calculate_stat(data)
        dim = len(data)
        bins = dim/10
        x_stat = np.linspace(np.min(data)-1,np.max(data)+1,bins)
        x_pln = np.linspace(np.floor(np.min(data)),np.ceil(np.max(data)),bins)
        
        plt.figure()
        plt.title("The error histogram")
        plt.hist(data, bins=int(bins), normed=True, alpha=1, histtype='stepfilled',
             color='steelblue', edgecolor='none')
        plt.ylabel("Frequency")
        plt.xlabel("Error in bins")
        Gd.plotGaussian(x_stat,mean_stat,var_stat,'v-g','mean=$\mu_{statis}$,var=$Sigma_{statis}$')
        Gd.plotGaussian(x_pln,self.mean_Pln,self.Sigma_Pln,'.-r','mean=$\mu_{%s}$, var=$Sigma_{%s}$'%(self.name,self.name))
        plt.legend()
        plt.title("Statistical histogram vs. planning mean and variance")

  
#if __name__ == '__main__':