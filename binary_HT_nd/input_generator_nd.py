# -*- coding: utf-8 -*-
"""
Created on Tue Oct 20 22:51:01 2020

@author: Zhaoliang
"""

import numpy as np
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd
import binary_HT_nd.Error_prob_cal as EprC

def generate_mean(nHy,nd,Range):
    means = []
    for i in range(nHy):
        mean = np.random.randint(Range[0],Range[1],size=nd)
        means.append(mean)
    return means 
"""        
def generate_cov(nHy,nd,Range):
    Sigmas = []
"""    

def input_generator_nd(nHy,nd,Range):
    #mean0 = np.random.rand(1,nd)
    means = generate_mean(nHy,nd,Range)
    return means
    
if __name__ == "__main__":
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([2]).reshape(1, 1)
    Sigma1 = np.array([3]).reshape(1, 1)    
    
    mean0 = np.array([0,0])
    mean1 = np.array([1,1])
    Sigma0 = np.array([[2,-1],[-1,2]]).reshape(2, 2)
    Sigma1 = np.array([[3,0.1],[0.1,3]]).reshape(2, 2)
    """
    mean0 = np.array([0,0,0])
    mean1 = np.array([1,1,1])
    Sigma0 = np.array([[2,-1,0],[0,2,0],[0,-1,2]]).reshape(3, 3)
    Sigma1 = np.array([[3,0.1,0],[0,3,0],[0,0.1,3]]).reshape(3, 3)
    """
    nHy = 2
    nd = 2
    Range = [0,5]
    mean = input_generator_nd(nHy,nd,Range)
    Sigma = np.random.randint(Range[0],Range[1],size=(3,3))
    Sigma1 = np.array([[3,0.9,0],[0,3,0],[0,0.9,3]]).reshape(3, 3)
    
    eyeM = np.eye(nd, dtype=int)
    randM = np.random.rand(nd,nd)
    randM.diagonal(offset=1)
    i = range(nd)
    print(i)
    randM[i, i] = 1