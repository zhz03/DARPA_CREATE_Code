# -*- coding: utf-8 -*-
"""
Created on Wed Sep  9 14:09:33 2020

@author: Zhaoliang
"""

def System_setup_generator():
    T = 101
    uts = [0,1]
    ts = [100,1]
    ut = [0,1]
    trials = 1000

    return T,uts,ts,ut,trials

if __name__ == '__main__':
    T,uts,ts,ut,trials = System_setup_generator()
