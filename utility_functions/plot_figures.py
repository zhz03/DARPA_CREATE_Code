# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 16:11:29 2020

@author: Zhaoliang
"""
import numpy as np
import matplotlib.pyplot as plt
import random
import random as rnd
from mpl_toolkits.mplot3d import Axes3D
from utility_functions.statis import multivariate_gaussian,plot_covariance_ellipse
import Gaussian_dist as Gd

""" 1d """
def plot_filter_error(xs,var=None):

    ys = xs
    xs = range(len(ys))
    #var = np.asarray(var)
    std = np.sqrt(var)
    std_top = ys+std
    std_btm = ys-std
    plt.plot(xs, ys+std, linestyle=':', color='k', lw=2,label='error bound of standard deviation')
    plt.plot(xs, ys-std, linestyle=':', color='k', lw=2)
    plt.fill_between(xs, std_btm, std_top,facecolor='orange', alpha=0.2)

def kf_plot_1d_1obj(data,label = 'observation',name = None):
    labelname = label + '_' + name
    if name == None:
        raise ValueError("Please specify which data you want to plot")
    if name == 'measurements':
        plt.scatter(range(len(data)), data,s=10, label = labelname)
    elif name == 'ground_truth':
        plt.plot(range(len(data)), data,marker='v',label = labelname)
    elif name == 'estimates':
        plt.plot(range(len(data)), data, 'r',label = labelname)
    
    
def kf_plot_1d(measurements,ground_truth,clr='r',mkr='v',label='observation1'):
    mea_name = label + '_measurement'
    gtru_name = label + '_ground_truth'
    plt.scatter(range(len(measurements)), measurements,color = clr,s=10, label = mea_name)
    plt.plot(range(len(ground_truth)), ground_truth,color=clr,marker=mkr,label = gtru_name)
    plt.legend()

def kf_plot_1d_loop(measurements,ground_truth,mkr='v',label='observation1'):
    mea_name = label + '_measurement'
    gtru_name = label + '_ground_truth'
    plt.plot(range(len(ground_truth)), ground_truth,marker=mkr,color='y',label = gtru_name)
    plt.scatter(range(len(measurements)), measurements,s=20,color='b',label = mea_name)
    
    plt.legend()    
    
def KF_plot(measurements,ground_truth,estimates,sigma,simulation=False):
    
    if simulation == False:
        plt.figure()
        plt.scatter(range(len(measurements)), measurements,s=10, label = 'Measurements')
        plt.plot(range(len(estimates)), np.array(estimates), 'r' ,label = 'Kalman Filter estimate')
        plt.xlabel('timesteps')
        plt.ylabel('physical quantity')
        plt.legend()
        plt.show()
    elif simulation == True:
        plt.figure()
        plot_filter_error(estimates, var=sigma)
        plt.plot(range(len(ground_truth)), ground_truth,'gv', label = 'ground_truth')
        plt.scatter(range(len(measurements)), measurements,s=20, label = 'Measurements')
        plt.plot(range(len(estimates)), np.array(estimates), 'r' ,label = 'Kalman Filter estimate')
        plt.xlabel('timesteps')
        plt.ylabel('physical quantity')
        plt.legend()
        plt.show()
        
def KF_plot2in1(measurements,ground_truth,estimates,measurements_act,ground_truth_act,estimates_act,timesteps):
    plt.figure()
    plt.plot(range(len(ground_truth)), ground_truth,'gv', label = 'ground_truth')
    plt.scatter(range(len(measurements)), measurements,s=20, label = 'Measurements')
    plt.plot(range(len(estimates)), np.array(estimates), 'r' ,label = 'Kalman Filter estimate')
    plt.plot(timesteps-1, ground_truth_act[-1],'kv', label = 'ground_truth_u=1')
    plt.scatter(timesteps-1, measurements_act[-1],color = 'k',s=20, label = 'Measurements_u=1')
    plt.plot([timesteps-2,timesteps-1], estimates_act[-2:], color='r', linewidth=3,label = 'Kalman Filter estimate_u=1')
    plt.xlabel('timesteps')
    plt.ylabel('physical quantity')
    plt.legend()
    plt.show()

""" nd """
def multiKf_plot(mea,gtru,esti = None,kfest_flag = False): 
    """
    mea: measurement data, input should be a n-d list
    gtru: ground truth data
    esti: kf estimates
    """
    n = len(mea)
    if kfest_flag == False:
        plt.figure()
        esti_name = '_KF estimate'
        for i in range(n):
            label = 'observation' + str(i)
            kf_plot_1d_loop(mea[i],gtru[i],mkr='v',label=label)  
        plt.xlabel('timesteps')
        plt.ylabel('physical quantity')
        plt.legend()
        plt.show()
    elif kfest_flag == True:
        plt.figure()
        esti_name = '_KF estimate'
        for i in range(n):
            label = 'observation' + str(i)
            kf_plot_1d_loop(mea[i],gtru[i],mkr='v',label=label)  
            plt.plot(range(len(esti[i])), esti[i], 'r' ,label = label + esti_name)
        plt.xlabel('timesteps')
        plt.ylabel('physical quantity')
        plt.legend()
        plt.show()

def multiKf_plot_dxdz(mea,gtru,esti = None,kfest_flag = False): 
    """
    mea: measurement data, input should be a n-d list
    gtru: ground truth data
    esti: kf estimates
    """
    m = len(mea)
    n = len(gtru)
    if kfest_flag == False:
        plt.figure()
        
        for i in range(m):
            label = 'observation' + str(i)
            kf_plot_1d_1obj(mea[i],label = label,name = 'measurements')
        for i in range(n):
            label = 'observation' + str(i)
            kf_plot_1d_1obj(gtru[i],label = label,name = 'ground_truth')
        plt.xlabel('timesteps')
        plt.ylabel('physical quantity')
        plt.legend()
        plt.show()
    elif kfest_flag == True:
        plt.figure()
        for i in range(m):
            label = 'observation' + str(i)
            kf_plot_1d_1obj(mea[i],label = label,name = 'measurements')
        for i in range(n):
            label = 'observation' + str(i)
            kf_plot_1d_1obj(gtru[i],label = label,name = 'ground_truth')
            kf_plot_1d_1obj(esti[i],label = label,name = 'estimates')
        plt.xlabel('timesteps')
        plt.ylabel('physical quantity')
        plt.legend()
        plt.show()
        
def plot_multi_var(mean1,mean0,U_sigma1,U_sigma0):
    dn = len(mean1)
    plt.figure()
    plot_covariance_ellipse(mean1,U_sigma1,fc='g', alpha=0.2, 
                        std=[1, 2, 3],
                        title='{%s}D Error(between ground truth and estimate) distribution'%dn)
    plot_covariance_ellipse(mean0,U_sigma0,fc='g', alpha=0.2, 
                    std=[1, 2, 3])
    plt.gca().grid(b=False)
    
def plot_1d_var(mean,U_sigma,newflag = True):
    if newflag == True:
        plt.figure()
        plot_covariance_ellipse(mean,U_sigma,fc='g', alpha=0.2, 
                            std=[1, 2, 3],
                            title='2D Error(between ground truth and estimate) distribution')
        plt.gca().grid(b=False)
    else:
        plot_covariance_ellipse(mean,U_sigma,fc='g', alpha=0.2, 
                            std=[1, 2, 3],
                            title='2D Error(between ground truth and estimate) distribution')
        plt.gca().grid(b=False)

def visulize_SM_data(System_model):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    num = len(System_model)
    for i in range(num):
        x = System_model[i]
        if i+2<=4:
            y = System_model[i+1]
            z = System_model[i+2]
        elif i+2>4 and i+1 <=4:
            y = System_model[i+1]
            z = System_model[i+2-5]
        elif i+1>4:
            y = System_model[i+1-5]
            z = System_model[i+2-5]
        if i ==0:
            label_name = 'A,H,B'
        elif i == 1:
            label_name = 'H,B,Q'
        elif i ==2:
            label_name = 'B,Q,R'
        elif i == 3:
            label_name = 'Q,R,A'
        elif i == 4:
            label_name = 'R,A,H'
        
        ax.scatter(x, y, z,color=(random.random(), random.random(), random.random()),label=label_name)
        plt.legend()
        #ax.set_title('',fontsize=12,color='k',horizontalalignment='center')
        
    plt.title("Parameters space")
    plt.show()

def plot_generate_seq_u(utsq,uts,ts):
    plt.figure()
    labelname1 = str(uts[0]) +';' + str(uts[1]) + ';' + str(uts[2])
    labelname2 = str(ts[0]) +';' + str(ts[1]) + ';' + str(ts[2])
    plt.plot(range(len(utsq)),utsq,label='uts=' + labelname1 +',' + 'ts=' +labelname2)
    plt.legend()

def plotGaussian(x,mean,var,Color,Label):
    y = Gd.gaussian(x, mean, np.sqrt(var))
    plt.plot(x, y,Color,label=Label)
    plt.legend()

def plot_2_Gaussian(mean0,mean1,var0,var1):
    x = Gd.generate_x(mean0,mean1,var0,var1)    
    plotGaussian(x,mean0,var0,'r.','mean={},var={}'.format(mean0,var0))
    plotGaussian(x,mean1,var1,'b.','mean={},var={}'.format(mean1,var1))  
    
if __name__ =="__main__":
    mean1 = [1,1]
    mean0 = [0,0]
    U_sigma1 =  np.array([[0.3,0.2],[0.2,0.3]]).reshape(2, 2)
    U_sigma0 =  np.array([[.3,0.0],[0.0,.3]]).reshape(2, 2)
    plot_multi_var(mean1,mean0,U_sigma1,U_sigma0)
    
    Sigma1 = [[1,0],[0,1]]
    Sigma2 = [[2.17,1.82], [1.82,2.17]]
    mu1 = [0,0]
    mu2 = [3,0]
    plot_multi_var(mu1,mu2,Sigma1,Sigma2)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = [[1],[2],[3],[4],[5]]
    y = [[1],[2],[3],[4],[5]]
    z = [[1],[2],[3],[4],[5]]
    ax.scatter(x, y, z,color=(random.random(), random.random(), random.random()),label="test")
    plt.legend()

    
    
    