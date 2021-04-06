# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 22:31:00 2020

@author: Zhaoliang and Zida 
"""

import copy
import numpy as np
from numpy.random import multivariate_normal
from scipy.linalg import eigh, cholesky
from scipy.stats import norm
import matplotlib.pyplot as plt
import Simulations.Generate_seq_u as Gsequ
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata

import E2Etest.SM_generator_1d as SMGen1d
import E2Etest.System_setup_generator as SSGen

np.set_printoptions(suppress=True)
np.set_printoptions(precision=20)

class Simulator_1t(object):
    def __init__(self, x0=0, ut=0,
                  A=1, B=1, H=1,R=0.0, Q=0.0):
        """ x0 - initial state 
            ut - (+=up, -=down)
            R: measurement_variance - variance in measurement 
            Q: process_variance - variance in process 
        """
        self.x = x0
        self.ut = ut
        self.measurement_var = R
        self.process_var = Q
        self.A = A
        self.B = B
        self.H = H
        self.n = A.shape[1]
        self.m = H.shape[1]  
        
    def convert_var2noise(self, M, dt=1.0):
        n = M.shape[1]
        rnd = norm.rvs(size=(n, int(dt)))
        # Compute the eigenvalues and eigenvectors.
        evals, evecs = eigh(M)
        # Construct c, so c*c^T = M.
        C = np.dot(evecs, np.diag(np.sqrt(evals)))
        # Convert the data to correlated random variables. 
        y = np.dot(C, rnd)
        return y
    def multi_noise(self,M):
        n = M.shape[1]
        mean = [0] * n
        y = multivariate_normal(mean,M)
        y = np.array(y).reshape(n,1)
        return y
    
    def state_transfer(self, dt=1.0):
        '''Compute new state'''
        # compute new state based on maneuver. Add in some
        # process noise
        #self.process_noise = self.convert_var2noise(self.process_var)
        self.process_noise = self.multi_noise(self.process_var)
        ut = np.dot(self.B,self.ut) +  self.process_noise
        self.x =np.dot(self.A,self.x)  + np.dot(ut, dt)
    def sensing(self):
        # simulate measuring the state with noise
        self.measurement_noise = self.convert_var2noise(self.measurement_var)
        return np.dot(self.H,self.x) + self.measurement_noise
    def transfer_and_sense(self, dt=1.0):
        self.state_transfer(dt)
        x = copy.deepcopy(self.x)
        return x, self.sensing()

    def run_simulation(self, dt=1, count=1):
        """ simulate the sensor measures data over a period of time.
        **Returns**
        data : np.array[float, float]
            2D array, first column contains actual state: the ground truth,
            second column contains the measurement of that state
        """
        # Compute the Cholesky decomposition.
        # We need a matrix `C` for which `C*C^T = R`.
        #C = cholesky(M, lower=True)
        return np.array([self.transfer_and_sense(dt) for i in range(count)])

def seperate_x_z(zs):
    gx = []
    zt = []
    for i in range(len(zs)):
        gx.append(zs[i][0])
        zt.append(zs[i][1])
    return gx,zt

def Simulator(SM,x0,ut_sq):
    A = SM[0]
    B = SM[1]
    H = SM[2]
    Q = SM[3]
    R = SM[4]
    timesteps = len(ut_sq)
    y = [] 
    z = []
    sim_1 = Simulator_1t(
        x0=x0, 
        ut=ut_sq[0], 
        R=R, 
        Q=Q, 
        A = A,
        B = B,
        H = H)
    for i in range(timesteps):
        if i==0:
            xzs_new = [sim_1.transfer_and_sense()]
            yt,zt = seperate_x_z(xzs_new)
            y.append(yt[0])
            z.append(zt[0])
        else:
            sim_1n = Simulator_1t(
                x0=y[-1], 
                ut=ut_sq[i], 
                R=R, 
                Q=Q, 
                A = A,
                B = B,
                H = H)
            xzs_new = [sim_1n.transfer_and_sense()]
            yt,zt = seperate_x_z(xzs_new)

        #    if yt[0][0] >= 1000000 or yt[0][0] <= -1000000 :
         #       yt[0][0] = yt[0][0]/1000000
        #        zt[0][0] = zt[0][0]/1000000
                
            if np.mean(yt[0]) >=10000000 or np.mean(yt[0]) <= -10000000:
                yt[0] /= 10000000
                zt[0] /= 10000000
            print("Info: y: {}, z: {}".format(yt[0], zt[0]) )
            y.append(yt[0])
            z.append(zt[0])
    return y,z

def figs_verification(num):
    dx = 1
    dz = 1
    #num = 2
    Arange = [0,2]
    Brange = [0,2]
    Hrange = [0,2]
    Qrange = [0,2]    
    Rrange = [0,2]
    System_models = SMGen1d.SM_generator_1d(num,Arange,Brange,Hrange,Qrange,Rrange)
    #As,Hs,Bs,Qs,Rs,
    T,uts,ts,ut,trials,x0 = SSGen.System_setup_generator()
    SM_num = len(System_models[0])
    for i in range(SM_num):
        A = System_models[0][i]
        B = System_models[1][i]
        H = System_models[2][i]
        Q = System_models[3][i]
        R = System_models[4][i]
        SM = [A,B,H,Q,R]
        u = Gsequ.generate_sequential_ut(uts,ts)
        y,z = Simulator(SM,x0,u)
        ground_truth = cnvdata.convert_array2list_nd(y,dx)
        measurements = cnvdata.convert_array2list_nd(z,dx)
        plotfgs.multiKf_plot(ground_truth,measurements)
        plt.title('A=' + '%.2f' % A + '; B=' + '%.2f' % B + '; H=' + '%.2f' % H + '; Q=' + '%.2f' % Q + '; R=' + '%.2f' % R)
        fig_name = './figs/simulator_figs/' + str(i) + '.jpg'
        plt.savefig(fig_name) 
        plt.close()
if __name__ == '__main__':
    figs_verification(20)
    """
    dx = 2
    dz = 2
    A = np.array([[1,0],[0,1]]).reshape(dx, dx)
    B = np.array([[1],[1]]).reshape(dx, 1) 
    H = np.array([[1,0],[0,1]]).reshape(dz, dx)
    Q = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(dx, dx)
    R = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(dz, dz)
    x0 = np.array([[0],[0]]).reshape(dx, 1)
    uts = [0,1,0,2]
    ts = [100,10,20,10]
    """
    """
    dx = 1
    dz = 1
    q = .1254
    r = 1.6798
    r = .1254
    a = 1.4816
    #a = 1.0
    h = 1.5927
    h = 1
    b = 1
    A = np.array([a]).reshape(dx, dx)
    H = np.array([h]).reshape(dz, dx)
    B = np.array([b]).reshape(dx, 1)
    Q = np.array([q]).reshape(dx, dx)
    R = np.array([r]).reshape(dz, dz)
    x0 = np.array([[0]]).reshape(dx, 1)
    
    uts = [0,1]
    ts = [100,1]
    ut = [0,1] 
    trials = 1000
    """

        
    """
    SM = [A,B,H,Q,R]
    y,z = Simulator(SM,x0,u)
    
    ground_truth = cnvdata.convert_array2list_nd(y,dx)
    measurements = cnvdata.convert_array2list_nd(z,dx)
    plotfgs.multiKf_plot(ground_truth,measurements)
    """
    