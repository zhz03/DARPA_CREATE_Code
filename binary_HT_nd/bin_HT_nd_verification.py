# -*- coding: utf-8 -*-
"""
Created on Wed Oct 14 16:28:10 2020

@author: Zhaoliang
"""

import random
import numpy as np
import binary_HT_nd.bin_HT_nd as bHT

def swap(a1,a2):
    if abs(a1) ==  abs(a2):
        a2 = a2 - 0.01
    if abs(a1) >  abs(a2):
        return abs(a1),a2
    else:        
        return abs(a2),a1

def generate_param(nd,range1):
    if nd == 1:
        m0 = round(random.uniform(range1[0],range1[1]))
        m1 = round(random.uniform(range1[0],range1[1]))
        s0 = abs(round(random.uniform(range1[0],range1[1])))
        s1 = abs(round(random.uniform(range1[0],range1[1])))
        
        mean0 = np.array([m0])
        mean1 = np.array([m1])
        Sigma0 = np.array([s0]).reshape(1, 1)
        Sigma1 = np.array([s1]).reshape(1, 1)
    elif nd == 2:
        m01 = round(random.uniform(range1[0],range1[1]))
        m11 = round(random.uniform(range1[0],range1[1]))
        m02 = round(random.uniform(range1[0],range1[1]))
        m12 = round(random.uniform(range1[0],range1[1]))
        mean0 = np.array([m01,m02])
        mean1 = np.array([m11,m12])
        
        s01 = round(random.uniform(range1[0],range1[1]))
        s02 = round(random.uniform(range1[0],range1[1]))
        s01,s02 = swap(s01,s02)
        s11 = round(random.uniform(range1[0],range1[1]))
        s12 = round(random.uniform(range1[0],range1[1]))
        s11,s12 = swap(s11,s12)    
        
        Sigma0 = np.array([[s01,s02],[s02,s01]]).reshape(2, 2)
        Sigma1 = np.array([[s11,s12],[s12,s11]]).reshape(2, 2)
        
    return mean0, mean1, Sigma0, Sigma1 
    
if __name__ == "__main__":
    """
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([1]).reshape(1, 1)
    Sigma1 = np.array([1]).reshape(1, 1)
    range1 = [-5,5]    
    """
    """
    mean0 = np.array([0,0])
    mean1 = np.array([1,1])
    Sigma0 = np.array([[2,-1],[-1,2]]).reshape(2, 2)
    Sigma1 = np.array([[3,0.1],[0.1,3]]).reshape(2, 2)
    
    mean0 = np.array([0,0,0])
    mean1 = np.array([1,1,1])
    Sigma0 = np.array([[2,-1,0],[0,2,0],[0,-1,2]]).reshape(3, 3)
    Sigma1 = np.array([[3,0.1,0],[0,3,0],[0,0.1,3]]).reshape(3, 3)
    """
    mean0, mean1, Sigma0, Sigma1 = generate_param(2,range1)
    Num_sam = 1000
    Pr_D,Pr_FA,Pr_M,Pr_CR = bHT.bin_HT_nd(mean0,mean1,Sigma0,Sigma1,Num_sam)