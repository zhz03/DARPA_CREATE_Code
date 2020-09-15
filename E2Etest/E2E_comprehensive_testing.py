# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 15:23:58 2020

@author: Zhaoliang
"""
import numpy as np
import E2Etest.E2E_validation as E2Eval
import E2Etest.SM_generator_1d as SMGen1d
import E2Etest.System_setup_generator as SSGen
import datetime

def E2E_com_test():
    System_models = SMGen1d.SM_generator_1d()
    #As,Hs,Bs,Qs,Rs,
    T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
    SM_num = len(System_models[0])
    
    Error_D = []
    Error_FA = []
    Error_M = []
    Error_CR = []
    for i in range(SM_num):
        A = System_models[0][i]
        B = System_models[1][i]
        H = System_models[2][i]
        Q = System_models[3][i]
        R = System_models[4][i]
        SM = [A,B,H,Q,R]
        """
        A = As[i]
        B = Bs[i]
        H = Hs[i]
        Q = Qs[i]
        R = Rs[i]
        """
        error_D,error_FA,error_M,error_CR = E2Eval.E2E_validation(SM,T,ut,x0,uts,ts,trials)
        
        Error_D.append(error_D)
        Error_FA.append(error_FA)
        Error_M.append(error_M)
        Error_CR.append(error_CR)
        
    return Error_D,Error_FA,Error_M,Error_CR

if __name__ == "__main__":
    starttime = datetime.datetime.now()

    Error_D,Error_FA,Error_M,Error_CR = E2E_com_test()
    np.save('3_Error_D.npy', Error_D)
    np.save('3_Error_FA.npy', Error_FA)
    np.save('3_Error_M.npy', Error_M)
    np.save('3_Error_CR.npy', Error_CR)
    endtime = datetime.datetime.now()
    print ((endtime - starttime).seconds)
