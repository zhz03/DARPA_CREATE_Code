# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 12:27:13 2020

@author: Zhaoliang
"""

import numpy as np
import multi_HT_nd.sampling_GDnd as smpl_GDnd
from scipy.stats import norm, multivariate_normal
from scipy.stats import multivariate_normal
from multi_HT_nd.input_generator_nd import input_generator_nd as InGen_nd 

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

def labelling_mH(means,Sigmas,Points):
    labelled_Points = []
    p_u = []
    H_num = len(means)
    p_num = len(Points)
    p_num_each = len(Points[0])
    for i in range(p_num):
        labelled_points = []
        for k in range(p_num_each):
            p_u = []
            for j in range(H_num):
                points = Points[i]
                prob = multivariate_normal(mean=means[j],cov=Sigmas[j]).pdf(points[k])
                p_u.append(prob)
            labelled_point = {'label':i,'data':points[k],'p_u':p_u}
            labelled_points.append(labelled_point)
        labelled_Points.append(labelled_points)            
    return labelled_Points

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

def label_comparison_mH(labelled_Points):
    labelled_Pts_Dec = []
    num = len(labelled_Points)
    p_num = len(labelled_Points[0])
    for i in range(num):
        labelled_pts_dec = []
        labelled_points = labelled_Points[i]
        
        for j in range(p_num):
            prob = labelled_points[j]['p_u']
            dec = prob.index(max(prob))
            labelled_points[j].update({'dec':dec})
            labelled_pts_dec.append(labelled_points[j])
        labelled_Pts_Dec.append(labelled_pts_dec)
    return labelled_Pts_Dec

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

def error_prob_count_mH(labelled_Pts_Dec):
    H_num = len(labelled_Pts_Dec)
    p_num = len(labelled_Pts_Dec[0])
    Prob = []
    
    for i in range(H_num):
        prob = []
        for j in range(H_num):
            p = 0
            prob.append(p)
        Prob.append(prob)
        
    for i in range(H_num):
        for j in range(p_num):
            label = labelled_Pts_Dec[i][j]['label']
            dec = labelled_Pts_Dec[i][j]['dec']
            if label == dec:
                Prob[i][label] = Prob[i][label] + 1
            else:
                Prob[i][dec] = Prob[i][dec] + 1
    for i in range(H_num):
        for j in range(H_num):
            Prob[i][j] = Prob[i][j]/p_num
            
    return Prob       
    

def Error_pro_cal(mean0,mean1,Sigma0,Sigma1,points):
    labelled_points = labelling(points,mean0,mean1,Sigma0,Sigma1)
    labelled_pts_dec = label_comparison(labelled_points)
    Pr_D,Pr_M,Pr_FA,Pr_CR = error_prob_count(labelled_pts_dec)
    return Pr_D,Pr_M,Pr_FA,Pr_CR               

def Error_pro_cal_mH(means,Sigmas,Points):
    labelled_points = labelling_mH(means,Sigmas,Points)
    labelled_pts_dec = label_comparison(labelled_points)
    Prob_errors = error_prob_count(labelled_pts_dec)
    return Prob_errors    
    
if __name__ == "__main__":
    """
    mean0 = np.array([0])
    mean1 = np.array([1])
    Sigma0 = np.array([1]).reshape(1, 1)
    Sigma1 = np.array([1]).reshape(1, 1)
    """
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
    nHy = 3
    nd = 2
    Range = [0,5]
    means,Sigmas = InGen_nd(nHy,nd,Range)    
    sam_size = 1000
    Points = smpl_GDnd.sampling_GDnd(means,Sigmas,sam_size)
    labelled_Points = labelling_mH(means,Sigmas,Points)
    labelled_Pts_Dec = label_comparison_mH(labelled_Points)
    Prob_error = error_prob_count_mH(labelled_Pts_Dec)
    
    #prob = labelled_Points[0][10]['p_u']
    