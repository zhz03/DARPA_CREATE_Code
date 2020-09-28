# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:50:59 2020

@author: Zhaoliang
"""
import utility_functions.plot_figures as plotfgs

def generate_sequential_ut(uts,ts):
    un = len(uts)
    ut_sq = []
    for i in range(un):
        t = ts[i]
        for j in range(t):
            ut_sq.append(uts[i])    
    return ut_sq

#def verification(uts,ts):    

if __name__ == '__main__':
    uts = [0,1,0]
    ts = [100,10,20]
    ut_sq = generate_sequential_ut(uts,ts)
    plotfgs.plot_generate_seq_u(ut_sq,uts,ts)
    