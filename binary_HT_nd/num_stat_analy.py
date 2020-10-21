# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 12:34:08 2020

@author: Zhaoliang
"""

import numpy as np
import binary_HT_nd.sampling_binGDnd_Plot as smpl_bGDnd_plt 

def num_stat_analy(Error_mean0,Error_mean1,Error_var0,Error_var1):
    Mean0_norms = []
    Mean1_norms = []
    Var0_norms = []
    Var1_norms = []
    
    num = len(Error_mean0)
    for i in range(num):
        mean0_norm = np.linalg.norm(Error_mean0[i])
        mean1_norm = np.linalg.norm(Error_mean1[i])
        var0_norm = np.linalg.norm(Error_var0[i])
        var1_norm = np.linalg.norm(Error_var1[i])
        Mean0_norms.append(mean0_norm)
        Mean1_norms.append(mean1_norm)
        Var0_norms.append(var0_norm)
        Var1_norms.append(var1_norm)
    return Mean0_norms,Mean1_norms,Var0_norms,Var1_norms

if __name__ == "__main__":
    filepath = './data_storage/verification_2d/'
    Error_mean0,Error_mean1,Error_var0,Error_var1 = smpl_bGDnd_plt.load_stat_data(filepath)    
    Mean0_norms,Mean1_norms,Var0_norms,Var1_norms = num_stat_analy(Error_mean0,Error_mean1,Error_var0,Error_var1)