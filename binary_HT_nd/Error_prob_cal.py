# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 12:27:13 2020

@author: Zhaoliang
"""

import numpy as np
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd
from scipy.stats import norm, multivariate_normal

def labelling(points,mean0,mean1,Sigma0,Sigma1):
    labeled_points = []
    num = len(points)
    for i in range(num):
        p0 = multivariate_normal(mean=mean0,cov=Sigma0).pdf(points[i])
        p1 = multivariate_normal(mean=mean1,cov=Sigma1).pdf(points[i])
        labeled_point = [points[i],p0,p1]
        labeled_points.append(labeled_point)
    return labeled_points


def label_comparison(label_points):
    num = len(label_points)
    for i in range(num):
        p0 = label_points[i][1]
        p1 = label_points[i][2]
        
        
def Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points,visualization_flg = False):

    count_CR = 0
    count_FA = 0
    count_M = 0
    count_D = 0
    num = len(points)
    if visualization_flg == True:
        Points_D = []
        Points_FA = []
        Points_M = []
        Points_CR = []
    for i in range(num):
        p0 = multivariate_normal(mean=mean0,cov=Sigma0).pdf(points[i])
        p1 = multivariate_normal(mean=mean1,cov=Sigma1).pdf(points[i])
        if i < num/2: 
            if p0 > p1:
                count_CR = count_CR + 1
            else:
                count_M = count_M + 1
        else: 
            if p0 < p1: 
                count_D = count_D + 1
            else:
                count_FA = count_FA + 1
        if visualization_flg == True:
            if i < num/2: 
                if p0 > p1:
                    points_CR = points[i]
                    #np.vstack(Points_CR,points_CR)
                    Points_CR.append(points_CR)
                else:
                    points_M = points[i]
                    Points_M.append(points_M)
            else:
                if p0 < p1: 
                    points_D = points[i]
                    Points_D.append(points_D)
                else:
                    points_FA = points[i]    
                    Points_FA.append(points_FA)                    
    
    Pr_D = count_D/(num/2)
    Pr_M = count_M/(num/2)
    Pr_FA = count_FA/(num/2)
    Pr_CR = count_CR/(num/2)
    if visualization_flg == False:
        return Pr_D,Pr_FA,Pr_M,Pr_CR
    else:
        return Pr_D,Pr_FA,Pr_M,Pr_CR,Points_D,Points_FA,Points_M,Points_CR
    
if __name__ == "__main__":
    """
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([1]).reshape(1, 1)
    Sigma1 = np.array([1]).reshape(1, 1)
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,1000)
    #Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points)
    Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points,False)
    #Pr_D,Pr_FA,Pr_M,Pr_CR,points_D,points_FA,points_M,points_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points,True)
    """
    mean0 = np.array([0,0])
    mean1 = np.array([1,1])
    Sigma0 = np.array([[2,-1],[-1,2]]).reshape(2, 2)
    Sigma1 = np.array([[3,0.1],[0.1,3]]).reshape(2, 2)
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,1000)
    #Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points)
    Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points,False)
    labeled_points = labelling(points,mean0,mean1,Sigma0,Sigma1)
    