# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 11:33:58 2020

@author: Zhaoliang
"""

import numpy as np
import utility_functions.plot_figures as plf
import random
import matplotlib.pyplot as plt

def SM_generator_1d_old():
    dx = 1
    dz = 1
    
    # system matrices parameters
    """
    qn = [0.1,0.3,0.5,0.7,0.9,1.0,1.3,1.6,1.8,2]
    rn = [0.1,0.3,0.5,0.7,0.9,1.0,1.3,1.6,1.8,2]
    an = [0.1,0.2,0.4,0.5,0.7,0.8,1,1.1,1.5,1.6,1.9,2.0]
    hn = [0.1,0.2,0.4,0.5,0.7,0.8,1,1.1,1.5,1.6,1.9,2.0]
    b = 1
    """
    # 2304
    
    an = [0.1] #[0.1,0.4,0.8,1,1.6,2.0]
    hn = [0.1,0.5,1,1.6,2.0] #[0.1,0.4,0.8,1,1.6,2.0]
    b = 1
    qn = [0.1,0.5,0.7,1.0,1.5,2]
    rn = [0.1,0.5,0.7,1.0,1.5,2]
    
    #test
    """
    an = [0.1,1.0]
    hn = [0.1,1.1]
    b = 1
    qn = [0.3,0.7]
    rn = [0.2,0.5]
    """
    As = []
    Hs = []
    Bs = []
    Qs = []
    Rs = []
    for i in range(len(an)):
        a = an[i]
        A = np.array([a]).reshape(dx, dx)
        
        for j in range(len(hn)):
            h = hn[j]
            H = np.array([h]).reshape(dz, dx)

            for k in range(len(qn)):
                q = qn[k]
                Q = np.array([q * q]).reshape(dx, dx)

                for m in range(len(rn)):
                    r = rn[m]
                    R = np.array([r * r]).reshape(dz, dz)
                    B = np.array([b]).reshape(dx, 1)
                    As.append(A)
                    Bs.append(B)
                    Hs.append(H)
                    Rs.append(R)
                    Qs.append(Q)


    # Stack a set of ABHQR as system models  (SM)
    SM = [As,Bs,Hs,Qs,Rs]
    return SM

def SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange):
    dx = 1
    dz = 1
    As = []
    Hs = []
    Bs = []
    Qs = []
    Rs = []    
    for i in range(num):
        A = np.array([random.uniform(Arange[0],Arange[1])]).reshape(dx,dx)
        B = np.array([random.uniform(Brange[0],Brange[1])]).reshape(dx,dx)
        H = np.array([random.uniform(Hrange[0],Hrange[1])]).reshape(dz,dx)
        Q = np.array([random.uniform(Qrange[0],Qrange[1])]).reshape(dx,dx)
        R = np.array([random.uniform(Rrange[0],Rrange[1])]).reshape(dz,dz)
        As.append(A)
        Bs.append(B)
        Hs.append(H)
        Rs.append(R)
        Qs.append(Q)        
    SM = [As,Bs,Hs,Qs,Rs]
    return SM    

def run_example():
    Arange = [0,4]
    Brange = [0,4]
    Hrange = [0,4]
    Qrange = [0,4]    
    Rrange = [0,4]
    num = 1000
    System_models = SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange)
    plf.visulize_SM_data(System_models)
    plt.savefig('./figs/SM_parameter_space.jpg')            
if __name__ == '__main__':
    
    """
    plf.visulize_SM_data(System_models)
    A = System_models[0][0]
    B = System_models[1][0]
    H = System_models[2][0]
    Q = System_models[3][0]
    R = System_models[4][0]
    SM = [A,B,H,Q,R]
    """
    """
    start = 0
    stop = 10
    step =0.1
    a = random.randrange(start, stop)
    b = random.random()
    ab = a * b
    """
    run_example()
    