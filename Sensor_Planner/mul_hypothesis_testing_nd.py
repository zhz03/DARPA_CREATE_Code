# -*- coding: utf-8 -*-
"""
Created on Wed Jan  6 17:43:18 2021

@author: Zhaoliang
"""

import numpy as np
from multi_HT_nd.input_generator_nd import input_generator_nd as InGen_nd
from multi_HT_nd.sampling_GDnd import sampling_GDnd as samGDnd
import multi_HT_nd.Error_prob_cal as EprobCal

def mul_Hyp_test_nd(ut,Sigma_ut,sam_size):
    Points = samGDnd(ut,Sigma_ut,sam_size)
    Prob_errors = EprobCal.Error_pro_cal_mH(ut,Sigma_ut,Points)
    return Prob_errors

if __name__ == "__main__":
    nHy = 3
    nd = 3
    Range = [0,5]
    means,Sigmas = InGen_nd(nHy,nd,Range)
    sam_size = 1000
    Prob_errors = mul_Hyp_test_nd(means,Sigmas,sam_size)