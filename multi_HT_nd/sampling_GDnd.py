# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 12:01:24 2020

@author: Zhaoliang
"""

import numpy as np
#from scipy.stats import multivariate_normal
from multi_HT_nd.input_generator_nd import input_generator_nd as InGen_nd 

def sampling_GDnd(means,Sigmas,sam_size):
    Points = []
    num = len(means)
    for i in range(num):
        mean_temp = np.array(means[i].flatten())
        cov_temp = Sigmas[i]
       # print("means[{}].flatten is {} and Sigmas[i].shape is {} and sam_size is {}".format(
       #     i, mean_temp.shape, cov_temp.shape, sam_size))
        points = np.random.multivariate_normal(mean=mean_temp ,cov=cov_temp ,size=1000)
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