# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 12:27:13 2020

@author: Zhaoliang
"""
from mpl_toolkits import mplot3d
import numpy as np
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd
from scipy.stats import norm, multivariate_normal
import matplotlib.pyplot as plt

def labelling(points,mean0,mean1,Sigma0,Sigma1):
    labelled_points = []
    num = len(points)
    for i in range(num):
        p0 = multivariate_normal(mean=mean0,cov=Sigma0).pdf(points[i])
        p1 = multivariate_normal(mean=mean1,cov=Sigma1).pdf(points[i])
        if i<num/2:
            #labelled_point = [mean0,points[i],p0,p1]
            labelled_point = {'label':0,'data':points[i],'p_u0':p0,'p_u1':p1}
        else:
            #labelled_point = [mean1,points[i],p0,p1]
            labelled_point = {'label':1,'data':points[i],'p_u0':p0,'p_u1':p1}
        labelled_points.append(labelled_point)
    return labelled_points


def label_comparison(labelled_points):
    labelled_pts_dec = []   
    num = len(labelled_points)
    for i in range(num):
        p0 = labelled_points[i]['p_u0']
        p1 = labelled_points[i]['p_u1']
        if p0 > p1:
            labelled_points[i].update({'dec':0})
        else:
            labelled_points[i].update({'dec':1})
        labelled_pts_dec.append(labelled_points[i])
    return labelled_pts_dec        

def error_prob_count(labelled_pts_dec):
    count_CR = 0
    count_FA = 0
    count_M = 0
    count_D = 0
    num = len(labelled_pts_dec)
    for i in range(num):
        if labelled_pts_dec[i]['label'] == 0:
            if labelled_pts_dec[i]['label'] == labelled_pts_dec[i]['dec']:
                count_CR = count_CR + 1
            else:
                count_FA = count_FA + 1
        elif labelled_pts_dec[i]['label'] == 1:
            if labelled_pts_dec[i]['label'] == labelled_pts_dec[i]['dec']:
                count_D = count_D + 1
            else:
                count_M = count_M + 1
    Pr_D = count_D/(num/2)
    Pr_M = count_M/(num/2)
    Pr_FA = count_FA/(num/2)
    Pr_CR = count_CR/(num/2)           
    return Pr_D,Pr_M,Pr_FA,Pr_CR 

def Error_pro_cal(mean0,mean1,Sigma0,Sigma1,points):
    labelled_points = labelling(points,mean0,mean1,Sigma0,Sigma1)
    labelled_pts_dec = label_comparison(labelled_points)
    Pr_D,Pr_M,Pr_FA,Pr_CR = error_prob_count(labelled_pts_dec)
    return Pr_D,Pr_M,Pr_FA,Pr_CR    
       
def Error_prob_cal_oldV(mean0,mean1,Sigma0,Sigma1,points,visualization_flg = False):

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
                count_FA = count_FA + 1
        else: 
            if p0 < p1: 
                count_D = count_D + 1
            else:
                count_M = count_M + 1
        if visualization_flg == True:
            if i < num/2: 
                if p0 > p1:
                    points_CR = points[i]
                    #np.vstack(Points_CR,points_CR)
                    Points_CR.append(points_CR)
                else:
                    points_FA = points[i]
                    Points_FA.append(points_FA)
            else:
                if p0 < p1: 
                    points_D = points[i]
                    Points_D.append(points_D)
                else:
                    points_M = points[i]    
                    Points_M.append(points_M)                    
    
    Pr_D = count_D/(num/2)
    Pr_M = count_M/(num/2)
    Pr_FA = count_FA/(num/2)
    Pr_CR = count_CR/(num/2)
    if visualization_flg == False:
        return Pr_D,Pr_M,Pr_FA,Pr_CR
    else:
        return Pr_D,Pr_M,Pr_FA,Pr_CR,Points_D,Points_M,Points_FA,Points_CR
    
if __name__ == "__main__":
    
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([1]).reshape(1, 1)
    Sigma1 = np.array([1]).reshape(1, 1)
    """
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,1000)
    #Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points)
    Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points,False)
    #Pr_D,Pr_FA,Pr_M,Pr_CR,points_D,points_FA,points_M,points_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points,True)
    """
    """
    mean0 = np.array([0,0])
    mean1 = np.array([1,1])
    Sigma0 = np.array([[2,-1],[-1,2]]).reshape(2, 2)
    Sigma1 = np.array([[3,0.1],[0.1,3]]).reshape(2, 2)
    """
    mean0 = np.array([0,0,0])
    mean1 = np.array([1,1,1])
    Sigma0 = np.array([[2,-1,0],[0,2,0],[0,-1,2]]).reshape(3, 3)
    Sigma1 = np.array([[3,2.9,0],[0,3,0],[0,2.9,3]]).reshape(3, 3)
    
    points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,1000)
    
    #Pr_D,Pr_FA,Pr_M,Pr_CR = Error_prob_cal(mean0,mean1,Sigma0,Sigma1,points)
    
    Pr1_D,Pr1_M,Pr1_FA,Pr1_CR = Error_prob_cal_oldV(mean0,mean1,Sigma0,Sigma1,points,False)
    Pr_D,Pr_M,Pr_FA,Pr_CR,Points_D,Points_M,Points_FA,Points_CR = Error_prob_cal_oldV(mean0,mean1,Sigma0,Sigma1,points,True)
    #Pr_D,Pr_M,Pr_FA,Pr_CR = Error_pro_cal(points,mean0,mean1,Sigma0,Sigma1)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    #for i in range(len(Points_CR)):
        #ax.scatter(Points_CR[i][0],Points_CR[i][0],Points_CR[i][0])
    ax.scatter(Points_CR[1][0],Points_CR[1][0],Points_CR[1][0])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')    
    plt.show()
