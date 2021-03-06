# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 16:42:36 2020

@author: Zhaoliang
"""

import numpy as np
import utility_functions.CompP2SHist as CompP2S
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import matplotlib.pyplot as plt        

def load_plt_data(filepath):
    #filepath = './data_storage/'
    
    filename1 = filepath + 'sample_points.npy'
    Sample_points = np.load(filename1) 
    
    filename2 = filepath + 'M0.npy'
    M0 = np.load(filename2)    
    
    filename3 = filepath + 'M1.npy'
    M1 = np.load(filename3)

    filename4 = filepath + 'S0.npy'
    S0 = np.load(filename4)  

    filename5 = filepath + 'S1.npy'
    S1 = np.load(filename5)
    
    return Sample_points,M0,M1,S0,S1    

def load_stat_data(filepath):
    #filepath = './data_storage/'
    filename1 = filepath + 'Error_mean0.npy'
    Error_mean0 = np.load(filename1)
    
    filename2 = filepath + 'Error_mean1.npy'
    Error_mean1 = np.load(filename2)
    
    filename3 = filepath + 'Error_var0.npy'
    Error_var0 = np.load(filename3)
    
    filename4 = filepath + 'Error_var1.npy'
    Error_var1 = np.load(filename4) 
    return Error_mean0,Error_mean1,Error_var0,Error_var1

def savefigs(title,fig_path,i = None,close_flg = True):
    plt.title(title)
    if i == None:
        fig_name = fig_path + title + '.jpg'
    else:
        fig_name = fig_path + str(i) + '.jpg'
    plt.savefig(fig_name)
    if close_flg == True:
        plt.close()
        
def verification_direct_results_plot1d(plt_sig):
    filepath = './data_storage/verification_1d/'
    Sample_points,M0,M1,S0,S1 = load_plt_data(filepath)
    
    if plt_sig == -1: #plot all the figures and save them
        trial_num = len(M0)
        for i in range(trial_num):
            points = Sample_points[i]
            m0 = M0[i]
            m1 = M1[i]
            s0 = S0[i]
            s1 = S1[i]
            
            num_sam = len(points)
            
            points0 = points[0:int(num_sam/2),:]
            points1 = points[int(num_sam/2):num_sam,:]
    
            compp2s = CompP2S.Compare_pln2statis_hist(mean_pln = None, Sigma_pln = None,name ='plan',bins = None,Range=None)
            
            plotfgs.plot_2_Gaussian(m0,m1,s0,s1)
            compp2s.visualization_self(points0,nflg = True,dataname ='stat0')
            compp2s.visualization_self(points1,nflg = True,dataname ='stat1')
            title = ''
            fig_path = './figs/verification_1d/'
            savefigs(title,fig_path,i,close_flg = True)
            
    else: # plot only one figure and save it
        i = plt_sig
        points = Sample_points[i]
        m0 = M0[i]
        m1 = M1[i]
        s0 = S0[i]
        s1 = S1[i]
        num_sam = len(points)
            
        points0 = points[0:int(num_sam/2),:]
        points1 = points[int(num_sam/2):num_sam,:]

        compp2s = CompP2S.Compare_pln2statis_hist(mean_pln = None, Sigma_pln = None,name ='plan',bins = None,Range=None)

        plotfgs.plot_2_Gaussian(m0,m1,s0,s1)
        compp2s.visualization_self(points0,nflg = True,dataname ='stat0')
        compp2s.visualization_self(points1,nflg = True,dataname ='stat1')
        title = ''
        fig_path = './figs/verification_1d/'
        savefigs(title,fig_path,i,close_flg = True)         

def verification_direct_results_plot2d(plt_sig):
    filepath = './data_storage/verification_2d/'
    Sample_points,M0,M1,S0,S1 = load_plt_data(filepath)
    if plt_sig == -1: #plot all the figures and save them
        trial_num = len(M0)
        for i in range(trial_num):
            points = Sample_points[i]
            m0 = M0[i]
            m1 = M1[i]
            s0 = S0[i]
            s1 = S1[i]
            
            num_sam = len(points)
            
            points0 = points[0:int(num_sam/2),:]
            points1 = points[int(num_sam/2):num_sam,:]
            
            plt.plot(points0[:, 0], points0[:, 1], 'ro')
            plt.plot(points1[:, 0], points1[:, 1], 'bo')
            title = 'Data' + str(i) + ' | verification plot'
            plotfgs.plot_multi_var2d(title, m0.reshape(2,1),m1.reshape(2,1),s0,s1)
            title = 'Verification for data ' + str(i) 
            fig_path = './figs/verification_2d/'
            savefigs(title,fig_path,i,close_flg = True)   
    else:
        i = plt_sig
        points = Sample_points[i]
        m0 = M0[i]
        m1 = M1[i]
        s0 = S0[i]
        s1 = S1[i]
        num_sam = len(points)
            
        points0 = points[0:int(num_sam/2),:]
        points1 = points[int(num_sam/2):num_sam,:]
        
        plt.plot(points0[:, 0], points0[:, 1], 'ro')
        plt.plot(points1[:, 0], points1[:, 1], 'bo')
        title = 'Data' + str(i) + ' | verification plot'
        plotfgs.plot_multi_var2d(title,m0.reshape(2,1),m1.reshape(2,1),s0,s1)
        
        title = 'Verification for data ' + str(i) 
        fig_path = './figs/verification_2d/'
        savefigs(title,fig_path,i,close_flg = True)   
    
    #return Sample_points,M0,M1,S0,S1

def verification_stat_results_plot1d():
    filepath = './data_storage/verification_1d/'
    Error_mean0,Error_mean1,Error_var0,Error_var1 = load_stat_data(filepath)
    compp2s = CompP2S.Compare_pln2statis_hist(mean_pln = None, Sigma_pln = None,name ='plan',bins = 100,Range=0.1)
    fig_path = './figs/stat_1d/'
    
    plt.figure()
    m_0,var_0 = compp2s.calculate_stat(Error_mean0)
    compp2s.visualization_self(Error_mean0,nflg = False,dataname ='Error_{mean0}',tname = 'mean_{stat} = %.6s, var_{stat} = %.6s' %(m_0,var_0))
    fig_name1 = fig_path + 'Error_mean0' + '.jpg'
    plt.savefig(fig_name1)
    
    plt.figure()
    m_1,var_1 = compp2s.calculate_stat(Error_mean1)
    compp2s.visualization_self(Error_mean1,nflg = False,dataname ='Error_{mean1}',tname = 'mean_{stat} = %.6s, var_{stat} = %.6s' %(m_1,var_1))
    fig_name2= fig_path + 'Error_mean1' + '.jpg'
    plt.savefig(fig_name2)
    
    plt.figure()
    m_0,var_0 = compp2s.calculate_stat(Error_mean0)
    compp2s.visualization_self(Error_var0,nflg = False,dataname ='Error_{var0}',tname = 'mean_{stat} = %.6s, var_{stat} = %.6s' %(m_0,var_0))
    fig_name3= fig_path + 'Error_var0' + '.jpg'
    plt.savefig(fig_name3)
    
    plt.figure()
    m_1,var_1 = compp2s.calculate_stat(Error_mean1)
    compp2s.visualization_self(Error_var1,nflg = False,dataname ='Error_{var1}',tname = 'mean_{stat} = %.6s, var_{stat} = %.6s' %(m_1,var_1))
    fig_name4= fig_path + 'Error_var1' + '.jpg'
    plt.savefig(fig_name4)
    
    return Error_mean0,Error_mean1,Error_var0,Error_var1
    
if __name__ == '__main__':
    #Sample_points,M0,M1,S0,S1 = load_plt_data()
    #Error_mean0,Error_mean1,Error_var0,Error_var1 = load_stat_data()    
    #verification_direct_results_plot1d(-1)
    #verification_direct_results_plot2d(-1)
    #Error_mean0,Error_mean1,Error_var0,Error_var1 = verification_stat_results_plot1d()
    
    filepath = './data_storage/verification_2d/'
    Error_mean0,Error_mean1,Error_var0,Error_var1 = load_stat_data(filepath)
    
    filepath = './data_storage/verification_1d/'
    Error_mean0,Error_mean1,Error_var0,Error_var1 = load_stat_data(filepath)    
    
    verification_stat_results_plot1d()
    """
    filepath = './data_storage/verification_2d/'
    Sample_points,M0,M1,S0,S1 = load_plt_data(filepath)
    Error_mean0,Error_mean1,Error_var0,Error_var1 = load_stat_data(filepath)
    m0 = M0[1]
    m1 = M1[1]
    s0 = S0[1]
    s1 = S1[1]
    """
    
    #plotfgs.plot_2_Gaussian(m0.reshape(2,1),m1.reshape(2,1),s0,s1)