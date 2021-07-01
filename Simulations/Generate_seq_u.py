# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:50:59 2020

@author: Zhaoliang and Zida 
"""
import utility_functions.plot_figures as plotfgs
import matplotlib.pyplot as plt
import random

def generate_sequential_ut(uts,ts):
    un = len(uts)
    ut_sq = []
    for i in range(un):
        t = ts[i]
        for j in range(t):
            ut_sq.append(uts[i])    
            # ut_sq.append(uts[random.randint(0, i)])  
    return ut_sq

def fig_verification(uts,ts,i):    
    ut_sq = generate_sequential_ut(uts,ts)
    plotfgs.plot_generate_seq_u(ut_sq,uts,ts)
    fig_name = './figs/Gen_seq_figs/' + str(i) + '.jpg'
    plt.savefig(fig_name)     

def single_randome_test():
    #initialize uts and ts 
    uts = [0,0,0]
    ts = [10,10,10]
    for i in range(3):
        uts[i] = round(random.uniform(0,4))
        ts[i] = round(random.uniform(10,100))
    fig_verification(uts,ts,i)  

def verification(num):
    for j in range(num):
        uts = [0,0,0]
        ts = [10,10,10]
        for i in range(3):
            uts[i] = round(random.uniform(0,4))
            ts[i] = round(random.uniform(10,100))
        fig_verification(uts,ts,j)

if __name__ == '__main__':
    #single_randome_test()
    verification(10)
    """
    uts = [0,1,2]
    ts = [100,10,20]
    ut_sq = generate_sequential_ut(uts,ts)
    plotfgs.plot_generate_seq_u(ut_sq,uts,ts)
    plt.savefig('./figs/Gen_seq_2.jpg') 
    """