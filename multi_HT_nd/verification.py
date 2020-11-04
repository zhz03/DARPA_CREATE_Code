# -*- coding: utf-8 -*-
"""
Created on Wed Nov  4 15:24:38 2020

@author: Zhaoliang
"""

import multi_HT_nd.input_generator_nd as iGennd
import multi_HT_nd.sampling_GDnd as samGDnd
import matplotlib.pyplot as plt

def visualize_sample_points(Points):
    num = len(Points)
    for i in range(num):
        points = Points[i]
        plt.scatter(points[:,0],points[:,1])
    plt.show()    
    
if __name__ == "__main__":
    nHy = 3
    nd = 2
    Range = [0,5]
    means,Sigmas = iGennd.input_generator_nd(nHy,nd,Range) 
    sam_size = 1000
    Points = samGDnd.sampling_GDnd(means,Sigmas,sam_size)
    visualize_sample_points(Points)