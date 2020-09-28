# -*- coding: utf-8 -*-
"""
Created on Wed Sep  9 14:09:33 2020

@author: Zhaoliang
"""
import numpy as np

def System_setup_generator():
    
    uts = [0,1]
    ts = [100,1]
    T = ts[0] + ts[1]
    ut = [0,1]
    trials = 1000
    dx = 1
    x0 = np.array([[0]]).reshape(dx, 1)
    return T,uts,ts,ut,trials,x0

if __name__ == '__main__':
    T,uts,ts,ut,trials,x0 = System_setup_generator()
