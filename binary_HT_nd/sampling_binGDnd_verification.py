# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 14:46:45 2020

@author: Zhaoliang
"""
import numpy as np
import random
import utility_functions.CompP2SHist as CompP2S
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import matplotlib.pyplot as plt
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd

def verification_1d(range1):
    m0 = round(random.uniform(range1[0],range1[1]))
    m1 = round(random.uniform(range1[0],range1[1]))
    mean0 = np.array([m0])
    mean1 = np.array([m1])
    s0 = abs(round(random.uniform(range1[0],range1[1])))
    s1 = abs(round(random.uniform(range1[0],range1[1])))
    Sigma0 = np.array([s0]).reshape(1, 1)
    Sigma1 = np.array([s1]).reshape(1, 1)
    num_sam = 1000
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,num_sam)
    points0 = points[0:num_sam,:]
    points1 = points[num_sam:num_sam * 2,:]
    
    compp2s0 = CompP2S.Compare_pln2statis_hist(mean_pln = m0, Sigma_pln = s0,name ='plan',bins = None,Range=None)
    #compp2s0.visualization_compare(points0,10)
    #compp2s1 = CompP2S.Compare_pln2statis_hist(mean_pln = m1, Sigma_pln = s1,name ='plan',bins = None,Range=None)
    #compp2s0.visualization_compare(points1,10)
    compp2s0.visualization_self(points0,nflg = True)
    compp2s0.visualization_self(points1,nflg = True)
    plotfgs.plot_2_Gaussian(m0,m1,s0,s1)
    return points,points0,points1
    
"""    
def sampling_binGDnd_verification(num,dim_type):
    if dim_type == 1:
"""

if __name__ == "__main__":
    range1 = [-10,10]
    points,points0,points1 = verification_1d(range1)