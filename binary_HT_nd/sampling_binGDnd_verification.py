# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 14:46:45 2020

@author: Zhaoliang
"""
import numpy as np
import random
import utility_functions.CompP2SHist as CompP2S
import binary_HT_nd.sampling_binGDnd as smpl_bGDnd    
import matplotlib.pyplot as plt

def swap(a1,a2):
    if abs(a1) ==  abs(a2):
        a2 = a2 - 0.01
    if abs(a1) >  abs(a2):
        return abs(a1),a2
    else:        
        return abs(a2),a1
    
def save_plt_data(filepath,Sample_points,M0,M1,S0,S1):
    
    #filepath = './data_storage/'
    
    filename1 = filepath + 'sample_points.npy'
    np.save(filename1, Sample_points) 
    
    filename2 = filepath + 'M0.npy'
    np.save(filename2, M0)    
    
    filename3 = filepath + 'M1.npy'
    np.save(filename3, M1)

    filename4 = filepath + 'S0.npy'
    np.save(filename4, S0)  

    filename5 = filepath + 'S1.npy'
    np.save(filename5, S1)  

def save_stat_files(filepath, Error_mean0,Error_mean1,Error_var0,Error_var1):
    
    #filepath = './data_storage/'
    
    filename1 = filepath + 'Error_mean0.npy'
    np.save(filename1, Error_mean0)
    
    filename2 = filepath + 'Error_mean1.npy'
    np.save(filename2, Error_mean1)
    
    filename3 = filepath + 'Error_var0.npy'
    np.save(filename3, Error_var0)
    
    filename4 = filepath + 'Error_var1.npy'
    np.save(filename4, Error_var1)    
    
def verification_1d(trial_num,num_sam,range1):
    Error_mean0 = []
    Error_var0 = []
    Error_mean1 = []
    Error_var1 = []
    Sample_points = []
    M0 = []
    M1 = []
    S1 = []
    S0 = []
    for i in range(trial_num):
        
        m0 = round(random.uniform(range1[0],range1[1]))
        m1 = round(random.uniform(range1[0],range1[1]))
        mean0 = np.array([m0])
        mean1 = np.array([m1])
        s0 = abs(round(random.uniform(range1[0],range1[1])))
        s1 = abs(round(random.uniform(range1[0],range1[1])))
        Sigma0 = np.array([s0]).reshape(1, 1)
        Sigma1 = np.array([s1]).reshape(1, 1)
        #num_sam = 1000
        points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,num_sam)
        
        # concatenate all the resutls to a variable
        Sample_points.append(points)
        M0.append(m0)
        M1.append(m1)
        S1.append(s1)
        S0.append(s0)
        
        points0 = points[0:num_sam,:]
        points1 = points[num_sam:num_sam * 2,:]
        compp2s = CompP2S.Compare_pln2statis_hist(mean_pln = None, Sigma_pln = None,bins = None,Range=None)
        mean_stat0,var_stat0 = compp2s.calculate_stat(points0)
        mean_stat1,var_stat1 = compp2s.calculate_stat(points1)
        error_mean0 = m0 - mean_stat0
        error_var0 = s0 - var_stat0
        error_mean1 = m1 - mean_stat1
        error_var1 = s1 - var_stat1
        
        # concatenate all the statistical comparison results
        Error_mean0.append(error_mean0)
        Error_mean1.append(error_mean1)
        Error_var0.append(error_var0)
        Error_var1.append(error_var1)
    
    filepath = './data_storage/verification_1d/'
    save_plt_data(filepath,Sample_points,M0,M1,S0,S1)
    save_stat_files(filepath,Error_mean0,Error_mean1,Error_var0,Error_var1)
    
    #return Sample_points,M0

def verification_2d(trial_num,num_sam,range1):
    Error_mean0 = []
    Error_var0 = []
    Error_mean1 = []
    Error_var1 = []
    Sample_points = []
    M0 = []
    M1 = []
    S1 = []
    S0 = []
    
    for i in range(trial_num): 
        m01 = round(random.uniform(range1[0],range1[1]))
        m11 = round(random.uniform(range1[0],range1[1]))
        m02 = round(random.uniform(range1[0],range1[1]))
        m12 = round(random.uniform(range1[0],range1[1]))
        mean0 = np.array([m01,m02])
        mean1 = np.array([m11,m12])
        
        s01 = round(random.uniform(range1[0],range1[1]))
        s02 = round(random.uniform(range1[0],range1[1]))
        s01,s02 = swap(s01,s02)
        s11 = round(random.uniform(range1[0],range1[1]))
        s12 = round(random.uniform(range1[0],range1[1]))
        s11,s12 = swap(s11,s12)    
        
        Sigma0 = np.array([[s01,s02],[s02,s01]]).reshape(2, 2)
        Sigma1 = np.array([[s11,s12],[s12,s11]]).reshape(2, 2)
        
        points = smpl_bGDnd.sampling_binGDnd(mean0,mean1,Sigma0,Sigma1,num_sam)
        
        Sample_points.append(points)
        M0.append(mean0)
        M1.append(mean1)
        S1.append(Sigma1)
        S0.append(Sigma0)
        
        points0 = points[0:int(num_sam),:]
        points1 = points[int(num_sam):num_sam*2,:]
        
        mean_stat0 = np.array([np.mean(points0[:,0]),np.mean(points0[:,1])])
        mean_stat1 = np.array([np.mean(points1[:,0]),np.mean(points1[:,1])])
        Sigma_stat0 = np.cov(points0.T)
        Sigma_stat1 = np.cov(points1.T)
        
        error_mean0 = mean0 - mean_stat0
        error_cov0 = Sigma0 - Sigma_stat0
        error_mean1 = mean1 - mean_stat1
        error_cov1 = Sigma1 - Sigma_stat1
        
        # concatenate all the statistical comparison results
        Error_mean0.append(error_mean0)
        Error_mean1.append(error_mean1)
        Error_var0.append(error_cov0)
        Error_var1.append(error_cov1)
    
    filepath = './data_storage/verification_2d/'    
    save_plt_data(filepath,Sample_points,M0,M1,S0,S1)
    save_stat_files(filepath,Error_mean0,Error_mean1,Error_var0,Error_var1)
    """
    plt.plot(points0[:, 0], points0[:, 1], 'ro')
    plt.plot(points1[:, 0], points1[:, 1], 'bo')
    plt.show()
    """
    #return points,points0,points1
    
"""    
def sampling_binGDnd_verification(num,dim_type):
    if dim_type == 1:
"""

if __name__ == "__main__":
    range1 = [-5,5]
    trial_num = 1000
    num_sam = 1000
    verification_1d(trial_num,num_sam,range1)
    verification_2d(trial_num,num_sam,range1)