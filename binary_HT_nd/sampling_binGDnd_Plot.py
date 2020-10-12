# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 16:42:36 2020

@author: Zhaoliang
"""

import numpy as np
             
def load_plt_data():
    filepath = './data_storage/'
    
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

def load_stat_data():
    filepath = './data_storage/'
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
        
def verification_direct_results_plot():
    
    points0 = points[0:num_sam,:]
    points1 = points[num_sam:num_sam * 2,:]
    
    compp2s = CompP2S.Compare_pln2statis_hist(mean_pln = None, Sigma_pln = None,name ='plan',bins = None,Range=None)
    #compp2s0.visualization_compare(points0,10)
    #compp2s1 = CompP2S.Compare_pln2statis_hist(mean_pln = m1, Sigma_pln = s1,name ='plan',bins = None,Range=None)
    #compp2s0.visualization_compare(points1,10)
    
    plotfgs.plot_2_Gaussian(m0,m1,s0,s1)
    compp2s.visualization_self(points0,nflg = True,dataname ='stat0')
    compp2s.visualization_self(points1,nflg = True,dataname ='stat1')
    
    title = ''
    fig_path = './figs/verification_1d/'
    savefigs(title,fig_path,i,close_flg = False)


if __name__ == '__main__':
    Sample_points,M0,M1,S0,S1 = load_plt_data()
    Error_mean0,Error_mean1,Error_var0,Error_var1 = load_stat_data() 
    