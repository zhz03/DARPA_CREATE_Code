# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 22:49:51 2020

@author: Zhaoliang
"""
import numpy as np
from scipy.stats import norm, multivariate_normal
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import matplotlib.pyplot as plt

def HT_2d(mean0,mean1,Sigma0,Sigma1):
    plotfgs.plot_multi_var(mean1,mean0,Sigma1,Sigma0)    

if __name__ == "__main__":
    
    mean0 = [0,0]
    mean1 = [1,1]
    Sigma0 = np.array([[2,-0.4],[-0.4,2]]).reshape(2, 2)
    Sigma1 = np.array([[.2,0.1],[0.1,.2]]).reshape(2, 2)
    plotfgs.plot_multi_var(mean1,mean0,Sigma1,Sigma0)
    

    xrange = np.arange(-1.5, 2.5, 0.01)
    yrange = np.arange(-1.5, 2.5, 0.01)
    xrange = np.arange(-2, 3, 0.01)
    yrange = np.arange(-2, 3, 0.01)
    xrange = np.arange(-2.5, 3.5, 0.01)
    yrange = np.arange(-2.5, 3.5, 0.01)
    xrange = np.arange(-5, 5, 0.01)
    yrange = np.arange(-5, 5, 0.01)
    Prob_D = 0
    Prob_FA = 0
    Prob_M = 0
    Prob_CR = 0
    Pn0 = 0
    Pn1 = 0
    intersectx = []
    intersecty = []
    for a in range(len(xrange)):
        for b in range(len(yrange)):
            point = [xrange[a],yrange[b]]
            p0 = multivariate_normal(mean=mean0,cov=Sigma0).pdf(point)
            p1 = multivariate_normal(mean=mean1,cov=Sigma1).pdf(point)
            Pn0 = Pn0 + p0
            Pn1 = Pn1 + p1
            if p0 < p1:
                Prob_D = Prob_D + p1
                Prob_FA = Prob_FA + p0
            if abs(p0 - p1) < 0.0001:
                intersectx.append(xrange[a])
                intersecty.append(xrange[b])
    Prob_D = Prob_D/Pn1
    Prob_FA = Prob_FA/Pn0
    Prob_M = 1 - Prob_D
    Prob_CR = 1 - Prob_FA
    
    plt.plot(intersectx,intersecty,'r.')