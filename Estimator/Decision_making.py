# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 00:08:35 2020

@author: LENOVO
"""
import numpy as np
from scipy.stats import multivariate_normal


def Decision_making(ut,ut_zt,mean_ut_zt,Sigma_ut_zt):
    
    if ( (len(mean_ut_zt) == 2) & (mean_ut_zt[0].shape[0] == 1)):
        Sigma_ut_zt0 = Sigma_ut_zt[0]
        Sigma_ut_zt1 = Sigma_ut_zt[1]
        mean0 = mean_ut_zt[0]
        mean1 = mean_ut_zt[1]
        ut0 = ut[0]
        ut1 = ut[1]
        #print("mean is {}, cov is {}. pdf is {} ".format(mean0, Sigma_ut_zt0,ut_zt  ))
        #print("1d: pdf ut|zt is ", ut_zt)
        pdf_H0 = multivariate_normal(mean=mean0,cov=Sigma_ut_zt0).pdf(ut_zt) 
        pdf_H1 = multivariate_normal(mean=mean1,cov=Sigma_ut_zt0).pdf(ut_zt) 
        if pdf_H0 >= pdf_H1:
            decide_u = ut0
        else: decide_u = ut1
    else:
        pdf_H_list = []
        for i in range(len(mean_ut_zt)):
            mean_cur = mean_ut_zt[i] # 1*n 
            Sigma_ut_zt_cur = Sigma_ut_zt[i] # scalar
            ut_zt_cur = ut_zt
            #dim_mean = mean_cur.shape[-1]
            #sigma = np.full((dim_mean,dim_mean), Sigma_ut_zt_cur)

            pdf_H_cur = multivariate_normal(mean=mean_cur.flatten() ,cov=(Sigma_ut_zt_cur)).pdf(ut_zt.flatten())
            pdf_H_list.append(pdf_H_cur)        
            
        max_pdf_H = pdf_H_list.index(max(pdf_H_list))
            
        decide_u  = ut[max_pdf_H]
        
    return decide_u

def Decision_making_MM(ut,ut_zt,mean_ut_zt,Sigma_ut_zt):
    
    if ( (len(mean_ut_zt) == 2) & (mean_ut_zt[0].shape[0] == 1)):
        Sigma_ut_zt0 = Sigma_ut_zt[0]
        Sigma_ut_zt1 = Sigma_ut_zt[1]
        mean0 = mean_ut_zt[0]
        mean1 = mean_ut_zt[1]
        ut0 = ut[0]
        ut1 = ut[1]

        pdf_H0 = multivariate_normal(mean=mean0,cov=Sigma_ut_zt0).pdf(ut_zt) 
        pdf_H1 = multivariate_normal(mean=mean1,cov=Sigma_ut_zt0).pdf(ut_zt) 
        if pdf_H0 >= pdf_H1:
            decide_u = ut0
        else: decide_u = ut1

    else:
        pdf_H_list = []
        for i in range(len(mean_ut_zt)):
            mean_cur = mean_ut_zt[i] # 1*n 
            Sigma_ut_zt_cur = Sigma_ut_zt[i] # scalar
            ut_zt_cur = ut_zt
            
            pdf_H_cur = multivariate_normal(mean=mean_cur.flatten() ,cov=(Sigma_ut_zt_cur)).pdf(ut_zt.flatten())
            pdf_H_list.append(pdf_H_cur)
          
        max_pdf_H = pdf_H_list.index(max(pdf_H_list))
            
        decide_u  = ut[max_pdf_H]
        
    return pdf_H_list, decide_u

if __name__ == '__main__':
    pass


    
