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
    
    def __init__(self, mean_pln = None, Sigma_pln = None,name = 'plan',bins = None,Range=None):
        #if(mean_pln is None or Sigma_pln is None):
        #    raise ValueError("Set proper statistics.")
        
        self.mean_Pln = 0 if mean_pln is None else mean_pln
        self.Sigma_Pln = 1 if Sigma_pln is None else Sigma_pln
        self.name = name
        self.bins = 100 if bins is None else bins
        self.Range = 1 if Range is None else Range
    
    def calculate_stat(self,data):
        mean_stat = np.mean(data)
        var_stat = np.var(data)
        return mean_stat,var_stat
    
    def numer_compare(self,data):
        mean_stat,var_stat = self.calculate_stat(data)
        mean_error = np.abs(mean_stat- self.mean_Pln)
        var_error = np.abs(var_stat - self.Sigma_Pln)
        return mean_error,var_error
    
    def visualization_compare(self,data,rate):
        mean_stat,var_stat = self.calculate_stat(data)
        dim = len(data)
        bins = dim/rate
        x_stat = np.linspace(np.min(data)-1,np.max(data)+1,bins)
        x_pln = np.linspace(np.floor(np.min(data)),np.ceil(np.max(data)),bins)
        
        plt.figure()
        plt.title("The error histogram")
        plt.hist(data, bins=int(bins), density=True, alpha=1, histtype='stepfilled',
             color='steelblue', edgecolor='none')
        plt.ylabel("Frequency")
        plt.xlabel("Error in bins")
        Gd.plotGaussian(x_stat,mean_stat,var_stat,'v-g','mean=$\mu_{statis}$,var=$Sigma_{statis}$')
        Gd.plotGaussian(x_pln,self.mean_Pln,self.Sigma_Pln,'.-r','mean=$\mu_{%s}$, var=$Sigma_{%s}$'%(self.name,self.name))
        plt.legend()
        plt.title("Statistical histogram vs. planning mean and variance")
        
    def visualization_self(self,data,nflg = False,dataname = 'data',tname = "Data histogram"):
        mean_stat,var_stat = self.calculate_stat(data)
        bins = self.bins
        x_stat = np.linspace(np.min(data)-self.Range,np.max(data)+self.Range,bins)
        
        plt.title(tname)
        #plt.legend(data,dataname)
        plt.hist(data, bins=int(bins), density=True, alpha=1, histtype='stepfilled',
             color='steelblue', edgecolor='none',label=dataname)
        plt.legend()
        plt.ylabel("Frequency")
        plt.xlabel("bins")
        if nflg == True:
            Gd.plotGaussian(x_stat,mean_stat,var_stat,'-g','mean=$\mu_{%s}$,var=$Sigma_{%s}$'%(dataname,dataname))    
        
  
if __name__ == '__main__':
    x = [rnd.random() for i in range(1000)]
    CpH = Compare_pln2statis_hist()
    CpH.visualization_self(x)
    y = [np.random.normal(0,1) for i in range(1000)]
    CpH.visualization_self(y,True)