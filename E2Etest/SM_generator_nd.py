# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 04:01:53 2021

@author: Zhaoliang and Zida 
"""
import numpy as np
from multi_HT_nd.input_generator_nd import generate_cov_new as GenCov

def SM_generator_nd_single(dx,du,dz,Arange,Brange,Hrange,Qrange,Rrange):
    
    A = np.random.uniform(Arange[0], Arange[1], (dx,dx))
    
    B = np.random.uniform(Brange[0], Brange[1], (dx,du))
    H = np.random.uniform(Hrange[0], Hrange[1], (dz,dx))
    Q = GenCov(Qrange,dx)[0]
    R = GenCov(Rrange,dz)[0]
    #Q = np.random.uniform(Qrange[0], Qrange[1], (dx,dx))
    #R = np.random.uniform(Rrange[0], Rrange[1], (dz,dz))
    SM = [A,B,H,Q,R]
    return SM

def SM_generator_nd(dx,dz,du,Arange,Brange,Hrange,Qrange,Rrange,num):
    As = []
    Hs = []
    Bs = []
    Qs = []
    Rs = []    
    for i in range(num):
        A = np.random.uniform(Arange[0], Arange[1], (dx,dx))
        B = np.random.uniform(Brange[0], Brange[1], (dx,du))
        H = np.random.uniform(Hrange[0], Hrange[1], (dz,dx))
        Q = GenCov(Qrange,dx)[0]
        R = GenCov(Rrange,dz)[0]
        As.append(A)
        Bs.append(B)
        Hs.append(H)
        Rs.append(R)
        Qs.append(Q)
    SM = [As,Bs,Hs,Qs,Rs]
    return SM

def SM_generator_constant(Qrange,Rrange,num):
    As = []
    Hs = []
    Bs = []
    Qs = []
    Rs = []    
    for i in range(num):
        A = np.load('As.npy')
        B = np.load('Bs.npy')
        H = np.load('Hs.npy')
        Q = np.load('Qs.npy')
        R = np.load('Rs.npy')
        As.append(A)
        Bs.append(B)
        Hs.append(H)
        Rs.append(R)
        Qs.append(Q)
    SM = [As,Bs,Hs,Qs,Rs]
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
    num = 100
    sm = SM_generator_nd(dx,dz,du,Arange,Brange,Hrange,Qrange,Rrange,num)