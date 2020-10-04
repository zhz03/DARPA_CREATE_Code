# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 15:34:45 2020

@author: Zhaoliang
"""

import numpy as np
import utility_functions.CompP2SHist as CompP2SHist

def M_Cov_Analy(data):
    mean = np.mean(data)
    var = np.var(data) 
    return mean,var
    
if __name__ =="__main__":

    Errors = []
    proname = 'M'
    for i in range(1,6):
        name = 'comprehensive_test_results/' + str(i) + '_Error_' + proname + '.npy'
        Error_i = np.load(name)
        for item in Error_i:
            Errors.append(item)

    x_mean,x_var = M_Cov_Analy(Errors)
    CompP2S = CompP2SHist.Compare_pln2statis_hist(Range=0.01)
    CompP2S.visualization_self(Errors,nflg = True)
    