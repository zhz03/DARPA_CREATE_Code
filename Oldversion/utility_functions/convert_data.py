# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 16:29:53 2020

@author: Zhaoliang
"""
import numpy as np
import matplotlib.pyplot as plt
import random as rnd

def convert_array2list_1d(data):
    l = len(data)
    data_l = []
    for i in range(l):
        data_l.append(data[i].tolist()[0][0])
    return data_l

def convert_array2list_nd(data,dim):
    l = len(data)
    data_nd = []
    for i in range(dim):
        data_l = []
        for j in range(l):
            data_l.append(data[j].tolist()[i][0])
        data_nd.append(data_l)
        j = 0
    return data_nd 

