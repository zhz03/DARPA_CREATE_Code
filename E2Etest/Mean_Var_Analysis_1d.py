# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 15:34:45 2020

@author: Zhaoliang
"""

import numpy as np

def M_Cov_Analy(data):
    mean = np.mean(data)
    var = np.var(data) 
    return mean,var
    
if __name__ =="__main__":
    x = [np.random.normal(0,1) for i in range(1000)]
    Error_D = np.load('Error_D.npy')
    x_mean,x_var = M_Cov_Analy(Error_D)