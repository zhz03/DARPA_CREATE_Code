# -*- coding: utf-8 -*-
"""
Created on Wed Sep  9 14:09:33 2020

@author: Zhaoliang
"""
import numpy as np
from multi_HT_nd.input_generator_nd import generate_mean as GenMean

def System_setup_generator():
    
    uts = [0,1]
    ts = [100,1]
    T = ts[0] + ts[1]
    ut = [0,1]
    trials = 1000
    dx = 1
    x0 = np.array([[0]]).reshape(dx, 1)
    return T,uts,ts,ut,trials,x0

def System_setup_generator_nd(nHy,nd,MRange):
    uts = GenMean(nHy,nd,MRange)
    ts = [100,1]
    T = ts[0] + ts[1]
    ut = uts
    trials = 1000   
    dx = nd
    x0 = np.random.uniform(0, 0, (dx,1))
    return T,uts,ts,ut,trials,x0
    
if __name__ == '__main__':
    #T,uts,ts,ut,trials,x0 = System_setup_generator()
    MRange = [0,3]
    nHy = 3
    nd = 2
    T,uts,ts,ut,trials,x0 = System_setup_generator_nd(nHy,nd,MRange)
