# -*- coding: utf-8 -*-
"""
Created on Tue Jul 21 14:39:27 2020

@author: Zhaoliang
"""
import numpy as np
import Sim_KF_Pln_nd.simulation as sim1
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import Sim_KF_Pln_nd.kalman_filter as kf
import Sim_KF_Pln_nd.planning as kfpln
import matplotlib.pyplot as plt

def analy_mean(means):
    """
    This function is to calculate quantitative mean outcomes
    """
    n = len(means)
    mean_init = [0,0]
    for i in range(n):
        mean_init = list(map(lambda x: x[0]+x[1], zip(mean_init, means[i] )))
    Mmean = list(map(lambda x: x[0]/n, zip(mean_init)))
    return Mmean

def analy_meandx(means,dx):
    """
    This function is to calculate quantitative mean outcomes
    for different dimention cases 
    """
    n = len(means)
    mean_init = np.zeros([1,dx]).reshape(1,dx)[0]
    for i in range(n):
        mean_init = list(map(lambda x: x[0]+x[1], zip(mean_init, means[i] )))
    Mmean = list(map(lambda x: x[0]/n, zip(mean_init)))
    return Mmean

def analy_var(Vars,dim):
    """
    This function is to calculate quantitative variance outcomes
    """
    n = len(Vars)
    Sum_Var = np.zeros((dim,dim))
    for i in range(n):
        Sum_Var = Sum_Var + Vars[i]
    M_var = Sum_Var/n
    return M_var   

def example_test_M22():
    """
    This is just one example to test 2d-state KF estimate
    By running this function, you'll see the visulization of these data
    """
    # specify dimension of dx and dz
    dx = 2
    dz = 2
    # specify the parameters of matrices
    q_std = 1
    q12 = .8
    r_std = 1
    r12 = 0.0
    a11 = 1
    a22 = 1
    a21 = 0
    a12 = 0.0 
    h11 = 1
    h22 = h11
    h12 = 0
    h21 = h12
    b1 = 1
    b2 = 1
    q21 = q12
    r21 = r12
    # set up variables to contain error mean and variance
    # between plan and statistics
    Emean_p_stat = []
    Evar_p_stat = []    
    # set up inputs
    A = np.array([[a11,a12],[a21,a22]]).reshape(dx, dx)
    B = np.array([[b1],[b2]]).reshape(dx, 1) 
    H = np.array([[h11,h12],[h21,h22]]).reshape(dz, dx)
    Q = np.array([[q_std * q_std,-q12 * q12],[-q21 * q21,q_std * q_std]]).reshape(dx, dx)
    R = np.array([[r_std * r_std,r12 * r12],[r21 * r21,r_std * r_std]]).reshape(dz, dz)
    x0 = np.array([[0],[0]]).reshape(dx, 1)
    uts = [0]
    ts = [1000]
    timesteps = np.sum(ts)
    # get sequential ut and simulated data from simulation
    ut_sq = sim1.generate_sequential_ut(uts,ts)
    ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq)
    # parse simulated data in KF
    predict,sigma,estimates = kf.KFprocess(A,B,H,Q,R,measurements)
    # calculate sigma from planning
    sigma_dr = kfpln.KF_planning(A, B, H, Q, R,timesteps)
    mean_dr = np.zeros([1,dx]).reshape(1,dx)[0]
    # convert data to a particular format
    ground_truth = cnvdata.convert_array2list_nd(ground_truth,dx)
    measurements = cnvdata.convert_array2list_nd(measurements,dz)
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    predict = cnvdata.convert_array2list_nd(predict,dx)
    # calculate the error between true states and estimated states
    error = [[ground_truth[i][j] - estimates[i][j] for j in range(timesteps)] for i in range(len(ground_truth))]

    mean_stats = []
    covar_stats = np.cov(error)
    for i in range(dx):
        meani = np.mean(error[i])
        mean_stats.append(meani)
    
    error_mean_p_stat = list(map(lambda x: x[0]-x[1], zip(mean_dr, mean_stats)))
    error_var_p_stat =  sigma_dr - covar_stats
    
    Emean_p_stat.append(error_mean_p_stat)
    Evar_p_stat.append(error_var_p_stat)
    
    plt.figure()
    plt.plot(error[0],error[1],'b*')
    plotfgs.plot_1d_var(mean_dr,sigma_dr,False)
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
    return Emean_p_stat,Evar_p_stat

def comprehensive_test_M22():
    """
    This is a comprehensive test of the different range of the system matrices in dx:2 dz:2 case
    """
    # specify dimension of dx and dz
    dx = 2
    dz = 2
    # specify the parameters of matrices
    q_std = 1
    q12 = 0.0
    r_std = 1
    r12 = 0.0
    q21 = q12
    r21 = r12
    a11 = 1
    a22 = 1
    a21 = 0
    a12 = 0.0 
    h11 = 1
    h22 = h11
    h12 = 0
    h21 = h12
    b1 = 1
    b2 = 1

    Tri = [0.1, 0.5,1,1.5,1.9] # test different parameters space
    trials = 1000 # run 1000 trials
    Total_analy_mean = []
    Total_analy_var = []
    """
    Some notes about Q and R matrices
    The diagonal terms >= off-diagonal terms
    """
    for ran in range(len(Tri)):
        Emean_p_stat = []
        Evar_p_stat = []
        Analy_mean = []
        Analy_var = []
        """
        You can change one element at a time 
        to do comprehensive matrix parameters space check 
        """
        #a11 = Tri[ran]
        #q_std = Tri[ran]
        #q12 = Tri[ran]
        #r12 = Tri[ran]
        h11 = Tri[ran]
        for sam in range(trials): # run 1000 trials
            r21 = r12
            q21 = q12
            a12 = a21
            a22 = a11
            h22 = h11
            h21 = h12
            
            A = np.array([[a11,a12],[a21,a22]]).reshape(dx, dx)
            B = np.array([[b1],[b2]]).reshape(dx, 1) 
            H = np.array([[h11,h12],[h21,h22]]).reshape(dz, dx)
            Q = np.array([[q_std * q_std,q12 * q12],[q21 * q21,q_std * q_std]]).reshape(dx, dx)
            R = np.array([[r_std * r_std,r12 * r12],[r21 * r21,r_std * r_std]]).reshape(dz, dz)
            x0 = np.array([[0],[0]]).reshape(dx, 1)
            
            uts = [0]
            ts = [1000] 
            timesteps = np.sum(ts)
            
            ut_sq = sim1.generate_sequential_ut(uts,ts)
            ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq) # need to debug
            predict,sigma,estimates = kf.KFprocess(A,B,H,Q,R,measurements)
            
            sigma_dr = kfpln.KF_planning(A, B, H, Q, R,timesteps)
            mean_dr = np.zeros([1,dx]).reshape(1,dx)[0]
            
            ground_truth = cnvdata.convert_array2list_nd(ground_truth,dx)
            measurements = cnvdata.convert_array2list_nd(measurements,dz)
            estimates = cnvdata.convert_array2list_nd(estimates,dx)
            predict = cnvdata.convert_array2list_nd(predict,dx)
            error = [[ground_truth[i][j] - estimates[i][j] for j in range(timesteps)] for i in range(len(ground_truth))]

            mean_stats = []
            covar_stats = np.cov(error)
            for i in range(dx):
                meani = np.mean(error[i])
                mean_stats.append(meani)
            
            error_mean_p_stat = list(map(lambda x: x[0]-x[1], zip(mean_dr, mean_stats)))
            error_var_p_stat =  sigma_dr - covar_stats
            
            Emean_p_stat.append(error_mean_p_stat)
            Evar_p_stat.append(error_var_p_stat)
            
        Analy_mean.append(analy_mean(Emean_p_stat))
        Analy_var.append(analy_var(Evar_p_stat,dx))
    Total_analy_mean = analy_mean(Analy_mean)
    Total_analy_var = analy_var(Analy_var,dx)
    print(Total_analy_mean)
    print(Total_analy_var)
    print("\n")
    return Emean_p_stat, Evar_p_stat

def example_test_M33():
    """
    This is just one example to test 3d-state KF estimate
    By running this function, you'll see the visulization of these data
    """
    # specify dimension of dx and dz
    dx = 3
    dz = 3
    # specify the parameters of matrices    
    a11 = 1
    a22 = 1
    a33 = 1
    a12 = 0
    a13 = 0
    a21 = 0
    a23 = 0
    a31 = 0
    a32 = 0
    h11 = 1
    h22 = 1
    h33 = 1
    h12 = 0
    h13 = 0
    h21 = 0
    h23 = 0
    h31 = 0
    h32 = 0
    rstd = 0.3
    r12 = 0.2
    r13 = 0.3
    r23 = 0.2
    qstd = 0.3
    q12 = 0
    q13 = 0
    q23 = 0
    # set up inputs
    A = np.array([[a11,a12,a13],[a21,a22,a23],[a31,a32,a33]]).reshape(dx, dx)
    B = np.array([[1],[1],[1]]).reshape(dx, 1) 
    H = np.array([[h11,h12,h13],[h21,h22,h23],[h31,h32,h33]]).reshape(dz, dx)
    Q = np.array([[qstd * qstd,q12*q12,q13*q13],[q12*q12,qstd*qstd,q23*q23],[q13*q13,q23*q23,qstd * qstd]]).reshape(dx, dx)
    R = np.array([[rstd * rstd,r12 * r12,r13*r13],[r12 * r12,rstd*rstd,r23*r23],[r13*r13,r23*r23,rstd * rstd]]).reshape(dz, dz)
    x0 = np.array([[0],[0],[0]]).reshape(dx, 1)  
    
    uts = [0]
    ts = [100]
    timesteps = np.sum(ts)
    
    Emean_p_stat = []
    Evar_p_stat = []
    # get sequential ut and simulated data from simulation
    ut_sq = sim1.generate_sequential_ut(uts,ts)
    ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq) # need to debug
    # parse simulated data in KF    
    predict,sigma,estimates = kf.KFprocess(A,B,H,Q,R,measurements)
    # calculate sigma from planning
    sigma_dr = kfpln.KF_planning(A, B, H, Q, R,timesteps)
    mean_dr = np.zeros([1,dx]).reshape(1,dx)[0]
    # convert data to a particular format
    ground_truth = cnvdata.convert_array2list_nd(ground_truth,dx)
    measurements = cnvdata.convert_array2list_nd(measurements,dz)
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    predict = cnvdata.convert_array2list_nd(predict,dx)
    # calculate the error between true states and estimated states
    error = [[ground_truth[i][j] - estimates[i][j] for j in range(timesteps)] for i in range(len(ground_truth))]

    mean_stats = []
    covar_stats = np.cov(error)
    for i in range(dx):
        meani = np.mean(error[i])
        mean_stats.append(meani)
    
    error_mean_p_stat = list(map(lambda x: x[0]-x[1], zip(mean_dr, mean_stats)))
    error_var_p_stat =  sigma_dr - covar_stats
    
    Emean_p_stat.append(error_mean_p_stat)
    Evar_p_stat.append(error_var_p_stat)
    plotfgs.multiKf_plot_dxdz(measurements,ground_truth,estimates,kfest_flag = True)
    return error_mean_p_stat,error_var_p_stat

def comprehensive_test_M33():  
    """
    This is a comprehensive test of the different range of 
    the system matrices in dx:3 dz:3 case
    """
    # specify dimension of dx and dz
    dx = 3
    dz = 3
    # specify the parameters of matrices    
    a11 = 1
    a22 = 1
    a33 = 1
    a12 = 0
    a13 = 0
    a21 = 0
    a23 = 0
    a31 = 0
    a32 = 0
    h11 = 1
    h22 = 1
    h33 = 1
    h12 = 0
    h13 = 0
    h21 = 0
    h23 = 0
    h31 = 0
    h32 = 0
    rstd = 0.3
    r12 = 0
    r13 = 0
    r23 = 0
    qstd = 1
    q12 = 0
    q13 = 0
    q23 = 0
    
    Tri = [0.1, 0.5,1,1.5,1.9]
    b1 = 1
    b2 = 1
    trials = 1000
    Total_analy_mean = []
    Total_analy_var = []
    for ran in range(len(Tri)):
        Emean_p_stat = []
        Evar_p_stat = []
        Analy_mean = []
        Analy_var = []
        #a21 = Tri[ran]
        q_std = Tri[ran]
        #q12 = Tri[ran]
        #r12 = Tri[ran]
        #q13 = Tri[ran]
        for sam in range(trials):
            #r12 = r13
            #q21 = q11
            #a33 = a11
            #q23 = q13
            # set up inputs
            A = np.array([[a11,a12,a13],[a21,a22,a23],[a31,a32,a33]]).reshape(dx, dx)
            B = np.array([[1],[1],[1]]).reshape(dx, 1) 
            H = np.array([[h11,h12,h13],[h21,h22,h23],[h31,h32,h33]]).reshape(dz, dx)
            Q = np.array([[qstd * qstd,q12*q12,q13*q13],[q12*q12,qstd*qstd,q23*q23],[q13*q13,q23*q23,qstd * qstd]]).reshape(dx, dx)
            R = np.array([[rstd * rstd,r12 * r12,r13*r13],[r12 * r12,rstd*rstd,r23*r23],[r13*r13,r23*r23,rstd * rstd]]).reshape(dz, dz)
            x0 = np.array([[0],[0],[0]]).reshape(dx, 1)  
            
            uts = [0]
            ts = [100] 
            timesteps = np.sum(ts)
            # get sequential ut and simulated data from simulation
            ut_sq = sim1.generate_sequential_ut(uts,ts)
            ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq) # need to debug
            # parse simulated data in KF   
            predict,sigma,estimates = kf.KFprocess(A,B,H,Q,R,measurements)
            
            sigma_dr = kfpln.KF_planning(A, B, H, Q, R,timesteps)
            mean_dr = np.zeros([1,dx]).reshape(1,dx)[0]
            
            ground_truth = cnvdata.convert_array2list_nd(ground_truth,dx)
            measurements = cnvdata.convert_array2list_nd(measurements,dz)
            estimates = cnvdata.convert_array2list_nd(estimates,dx)
            predict = cnvdata.convert_array2list_nd(predict,dx)
            error = [[ground_truth[i][j] - estimates[i][j] for j in range(timesteps)] for i in range(len(ground_truth))]
        
            mean_stats = []
            covar_stats = np.cov(error)
            for i in range(dx):
                meani = np.mean(error[i])
                mean_stats.append(meani)
            
            error_mean_p_stat = list(map(lambda x: x[0]-x[1], zip(mean_dr, mean_stats)))
            error_var_p_stat =  sigma_dr - covar_stats
            
            Emean_p_stat.append(error_mean_p_stat)
            Evar_p_stat.append(error_var_p_stat)            
            
        Analy_mean.append(analy_meandx(Emean_p_stat,dx))
        Analy_var.append(analy_var(Evar_p_stat,dx))
    Total_analy_mean = analy_meandx(Analy_mean,dx)
    Total_analy_var = analy_var(Analy_var,dx)
    print(Total_analy_mean)
    print(Total_analy_var)
    print("\n")
    return Emean_p_stat, Evar_p_stat


def comprehensive_test_H_dx2dz3():
    """
    This is a comprehensive test of the different range of 
    the H matrix in dx:2 dz:3 case.
    If you want to try dx:2,dz:1 case
    then you'll need to change line 455 to
    H = np.array([[h11,h12]]).reshape(dz, dx)
    and dz = 1
    """    
    dx = 2
    dz = 3
    
    q_std = 1
    q12 = 0
    r_std = 1
    r12 = 0
    r13 = 0
    r23 = 0
    a11 = 1
    a12 = 0.0
    a21 = a12 
    h11 = 1
    h22 = h11
    h32 = 1
    h31 = 0
    h21 = 0
    b1 = 1
    b2 = 1
    
    Htri = [0.1,0.5,0.7,1,2,3,4]
    trials = 1000
    Total_analy_mean = []
    Total_analy_var = []
    for ran in range(len(Htri)):
        Emean_p_stat = []
        Evar_p_stat = []
        Analy_mean = []
        Analy_var = []
        h12 = Htri[ran]
        for sam in range(trials):
            q21 = q12
            a12 = a21
            a22 = a11
            h22 = h11
            h21 = h12
            h31 = h12
            
            A = np.array([[a11,a12],[a21,a22]]).reshape(dx, dx)
            B = np.array([[b1],[b2]]).reshape(dx, 1) 
            H = np.array([[h11,h12],[h21,h22],[h31,h32]]).reshape(dz, dx)
            Q = np.array([[q_std * q_std,q12 * q12],[q21 * q21,q_std * q_std]]).reshape(dx, dx)
            R = np.array([[r_std * r_std,r12 * r12,r13*r13],[r12 * r12,r_std * r_std,r23 * r23],[r13*r13,r23 * r23,r_std * r_std]]).reshape(dz, dz)
            x0 = np.array([[0],[0]]).reshape(dx, 1)
            
            uts = [0]
            ts = [1000] 
            timesteps = np.sum(ts)
            
            ut_sq = sim1.generate_sequential_ut(uts,ts)
            ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq) # need to debug
            predict,sigma,estimates = kf.KFprocess(A,B,H,Q,R,measurements)
            
            sigma_dr = kfpln.KF_planning(A, B, H, Q, R,timesteps)
            mean_dr = np.zeros([1,dx]).reshape(1,dx)[0]
            
            ground_truth = cnvdata.convert_array2list_nd(ground_truth,dx)
            measurements = cnvdata.convert_array2list_nd(measurements,dz)
            estimates = cnvdata.convert_array2list_nd(estimates,dx)
            predict = cnvdata.convert_array2list_nd(predict,dx)
            error = [[ground_truth[i][j] - estimates[i][j] for j in range(timesteps)] for i in range(len(ground_truth))]
        
            mean_stats = []
            covar_stats = np.cov(error)
            for i in range(dx):
                meani = np.mean(error[i])
                mean_stats.append(meani)
            
            error_mean_p_stat = list(map(lambda x: x[0]-x[1], zip(mean_dr, mean_stats)))
            error_var_p_stat =  sigma_dr - covar_stats
            
            Emean_p_stat.append(error_mean_p_stat)
            Evar_p_stat.append(error_var_p_stat)
            
        Analy_mean.append(analy_mean(Emean_p_stat))
        Analy_var.append(analy_var(Evar_p_stat,dx))
    Total_analy_mean = analy_mean(Analy_mean)
    Total_analy_var = analy_var(Analy_var,dx)
    print(Total_analy_mean)
    print(Total_analy_var)
    print("\n") 
    return Emean_p_stat, Evar_p_stat

if __name__ == '__main__':
    """
    The function name that starts with comprehensive_test_xxx
    will be runing slow, please be patient.
    Comprehensive test will run 1000 trials across 
    all test parameters space. 
    """
    error_mean_p_stat,error_var_p_stat = example_test_M22()
    #Emean_p_stat, Evar_p_stat = comprehensive_test_M22()
    #example_test_M33()
    #Emean_p_stat, Evar_p_stat = comprehensive_test_M33()
    #Emean_p_stat, Evar_p_stat = example_test_H()
    #Emean_p_stat, Evar_p_stat = comprehensive_test_H_dx2dz3()