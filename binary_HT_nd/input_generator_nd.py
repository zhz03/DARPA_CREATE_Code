# -*- coding: utf-8 -*-
"""
Created on Tue Oct 20 22:51:01 2020

@author: Zhaoliang
"""

import numpy as np
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd
import binary_HT_nd.Error_prob_cal as EprC
import random

def generate_mean(nHy,nd,Range):
    means = []
    for i in range(nHy):
        mean = np.random.randint(Range[0],Range[1],size=nd)
        means.append(mean)
    return means 
    
def generate_cov(nHy,nd):
    Sigmas = []
    for i in range(nHy):
        # create a right Sigma
        # To generate a uniform distribution random matrix over [-0.99,1)
        a = -0.9999
        b = 1
        Sigma = (b - a) * np.random.random_sample((nd,nd)) + a     
        j = range(nd)
        Sigma[j,j] = 1 
        Sigma = np.triu(Sigma)
        Sigma += Sigma.T - np.diag(Sigma.diagonal())
        Sigmas.append(Sigma)
    return Sigmas

def input_generator_nd(nHy,nd,Range):
    #mean0 = np.random.rand(1,nd)
    means = generate_mean(nHy,nd,Range)
    Sigmas = generate_cov(nHy,nd)
    return means,Sigmas
    
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
    nd = 5
    Range = [0,5]
    means,Sigmas = input_generator_nd(nHy,nd,Range)
