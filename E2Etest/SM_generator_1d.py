# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 11:33:58 2020

@author: Zhaoliang
"""

import numpy as np
import random

def inputs_generator_1d():
    dx = 1
    dz = 1
    # system matrices parameters
    """
    qn = [0.1,0.3,0.5,0.7,0.9,1.0,1.3,1.6,1.8,2,3,4]
    rn = [0.1,0.3,0.5,0.7,0.9,1.0,1.3,1.6,1.8,2,3,4]
    an = [0.1,0.2,0.4,0.5,0.7,0.8,1,1.1,1.5,1.6,1.9,2.0]
    hn = [0.1,0.2,0.4,0.5,0.7,0.8,1,1.1,1.5,1.6,1.9,2.0]
    bn = 1
    """
    # 2304
    """
    an = [0.1,0.4,0.8,1,1.6,2.0]
    hn = [0.1,0.4,0.8,1,1.6,2.0]
    b = 1
    qn = [0.1,0.5,0.7,1.0,1.5,2,3,4]
    rn = [0.1,0.5,0.7,1.0,1.5,2,3,4]
    """
    #test
    an = [0.1,1.0]
    hn = [0.1,1.1]
    b = 1
    qn = [0.1,0.7]
    rn = [0.1,0.5]
    
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
    SM = [As,Hs,Bs,Qs,Rs]
    return SM,T,uts,ts,ut
    
if __name__ == '__main__':
    System_model,T,uts,ts,ut = inputs_generator_1d()

    
    """
    start = 0
    stop = 10
    step =0.1
    a = random.randrange(start, stop)
    b = random.random()
    ab = a * b
    """