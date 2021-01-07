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

def generate_cov_new(Range,nd,nHy = None):
    Sigmas = []
    nHy = 1 if nHy is None else nHy
    for i in range(nHy):
        b = np.max(Range)
        a = -np.max(Range)
        Sigma = (b - a) * np.random.random_sample((nd,nd)) + a
        Sigma_b = np.dot(Sigma,Sigma.transpose())
        Sigma_c = Sigma_b+Sigma_b.T # makesure symmetric
        Sigmas.append(Sigma_c)
    return Sigmas        
    
    
def input_generator_nd(nHy,nd,Range):
    #mean0 = np.random.rand(1,nd)
    means = generate_mean(nHy,nd,Range)
    Sigmas = generate_cov_new(Range,nd,nHy)
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
    nHy = 4
    nd = 3
    Range = [0,5]
    means,Sigmas = input_generator_nd(nHy,nd,Range)
 
    """
    matrixSize = 3 
    A = np.random.rand(matrixSize,matrixSize)
    B = np.dot(A,A.transpose())
    C = B+B.T # makesure symmetric
    # test whether C is definite
    D = np.linalg.cholesky(C)

    
    math = [84, 82, 81, 89, 73, 94, 92, 70, 88, 95]
    science = [-85, -82, -72, -77, -75, -89, -95, -84, -77, -94]
    history = [97, 94, 93, 95, 88, 82, 78, 84, 69, 78]
    
    cov = np.cov([math,science,history])
    cov_unit = cov/ np.max(cov)
    
    b = 1
    a = -1
    nd = 3
    Sigma = (b - a) * np.random.random_sample((nd,nd)) + a
    Sigma_b = np.dot(Sigma,Sigma.transpose())
    Sigma_c = Sigma_b+Sigma_b.T # makesure symmetric
    """