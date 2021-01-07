# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 04:01:53 2021

@author: Zhaoliang
"""
import numpy as np

def SM_generator_nd(dx,dz,du,Arange,Brange,Hrange,Qrange,Rrange):
    
    A = np.random.uniform(Arange[0], Arange[1], (dx,dx))
    B = np.random.uniform(Brange[0], Brange[1], (dx,du))
    H = np.random.uniform(Hrange[0], Hrange[1], (dz,dx))
    Q = np.random.uniform(Qrange[0], Qrange[1], (dx,dx))
    R = np.random.uniform(Rrange[0], Rrange[1], (dz,dz))
    SM = [A,B,H,Q,R]
    return SM

if __name__ == "__main__":
    dx = 3
    dz = 2
    du = 1
    Arange = [0,4]
    Brange = [0,4]
    Hrange = [0,4]
    Qrange = [0,4]    
    Rrange = [0,4]
    sm = SM_generator_nd(dx,dz,du,Arange,Brange,Hrange,Qrange,Rrange)