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
        print("1d: decide u is", decide_u)
    else:
        pdf_H_list = []
        for i in range(len(mean_ut_zt)):
            mean_cur = mean_ut_zt[i] # 1*n 
            Sigma_ut_zt_cur = Sigma_ut_zt[i] # scalar
            
            #dim_mean = mean_cur.shape[-1]
            #sigma = np.full((dim_mean,dim_mean), Sigma_ut_zt_cur)
            print("mean is {}, cov is {}. pdf is {}".format(mean_cur.flatten(),int(Sigma_ut_zt_cur),  ut_zt.flatten()))
            #print("nd: pdf ut|zt is ", ut_zt)
            pdf_H_cur = multivariate_normal(mean=mean_cur.flatten() ,cov=int(Sigma_ut_zt_cur)).pdf(ut_zt.flatten())
            pdf_H_list.append(pdf_H_cur)
  #          print("mean.dim is {}, cov.dim is {}, ut_dim is {}, pdf.dim is {}, pdf_H_cur".
  #                format(mean_cur.flatten().shape, Sigma_ut_zt_cur.shape, 
  #                       ut_zt.T.shape, pdf_H_cur.shape, pdf_H_cur)
  #                )          
            
        max_pdf_H = pdf_H_list.index(max(pdf_H_list))
            
        decide_u  = ut[max_pdf_H]
        print("nd: decide u is", decide_u)
        
    return decide_u

if __name__ == '__main__':
    pass


    
