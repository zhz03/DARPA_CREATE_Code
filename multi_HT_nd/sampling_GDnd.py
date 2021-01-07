# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 12:01:24 2020

@author: Zhaoliang
"""

import numpy as np
from scipy.stats import multivariate_normal
from multi_HT_nd.input_generator_nd import input_generator_nd as InGen_nd 

def sampling_GDnd(means,Sigmas,sam_size):
    Points = []
    num = len(means)
    for i in range(num):
        points = np.random.multivariate_normal(mean = means[i],cov= Sigmas[i],size = sam_size)
        Points.append(points)
    #points0 = np.random.multivariate_normal(mean=mean0, cov=Sigma0, size=sam_size)
    #points1 = np.random.multivariate_normal(mean=mean1, cov=Sigma1, size=sam_size)
    
    #points = np.vstack((points0,points1)) 

    return Points

if __name__ == "__main__":
    nHy = 3
    nd = 3
    Range = [0,5]
    means,Sigmas = InGen_nd(nHy,nd,Range)    
    sam_size = 1000
    Points = sampling_GDnd(means,Sigmas,sam_size)