# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 21:55:57 2020

@author: Zhaoliang
"""

import numpy as np

import Sensor_Planner.KF_planner_nd as kfplnnd
import Sensor_Planner.Ut_planner_nd as Uplnnd
import binary_HT_1d.Gaussian_dist as Gd

def Bin_Hyp_test_1d(Sigma_ut,ut):
    Sigma_ut0 = Sigma_ut[0]
    Sigma_ut1 = Sigma_ut[1]
    ut0 = ut[0]
    ut1 = ut[1]

    Prob_D,Prob_FA,Prob_M,Prob_CR = Gd.error_prob(ut0,ut1,Sigma_ut0,Sigma_ut1)
    return Prob_D,Prob_FA,Prob_M,Prob_CR

if __name__ == "__main__":
    Sigma_ut = [0.1,0.1]
    ut = [0,1]
    Prob_D,Prob_FA,Prob_M,Prob_CR = Bin_Hyp_test_1d(Sigma_ut,ut)