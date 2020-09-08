# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 01:26:02 2020

@author: Zhaoliang
"""


def Error_prob_Comp(Pr_D,Pr_FA,Pr_M,Pr_CR,Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat):
    error_D = Pr_D_stat - Pr_D
    error_FA = Pr_FA_stat - Pr_FA
    error_M = Pr_M_stat - Pr_M
    error_CR = Pr_CR_stat - Pr_CR
    return error_D,error_FA,error_M,error_CR


if __name__ == '__main__':
    Pr_D,Pr_FA,Pr_M,Pr_CR = 0.9,0.11,0.1,0.89
    Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat = 0.89,0.1,0.11,0.9
    error_D,error_FA,error_M,error_CR = Error_prob_Comp(Pr_D,Pr_FA,Pr_M,Pr_CR,Pr_D_stat,Pr_FA_stat,Pr_M_stat,Pr_CR_stat)
