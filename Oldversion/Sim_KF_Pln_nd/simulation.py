# -*- coding: utf-8 -*-
"""
Created on Fri Jul  3 14:47:56 2020

@author: Zhaoliang
"""

import copy
import numpy as np
from numpy.random import multivariate_normal
from scipy.linalg import eigh, cholesky
from scipy.stats import norm
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata

class Simulation(object):
    def __init__(self, x0=0, maneuver=0,
                  A=1, B=1, H=1,R=0.0, Q=0.0):
        """ x0 - initial state 
            maneuver - (+=up, -=down)
            R: measurement_variance - variance in measurement 
            Q: process_variance - variance in process 
        """
        self.x = x0
        self.maneuver = maneuver
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
        maneuver = np.dot(self.B,self.maneuver) +  self.process_noise
        self.x =np.dot(self.A,self.x)  + np.dot(maneuver, dt)
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

def generate_fake(A,B,H,Q,R,x0,timesteps,action=0):
    """
    This function is to generate multi-dimensional simulated data
    A - system transfer matrix:  dx * dx
    B - input matrix: dx * 1
    H - observation matrix: dz * dx 
    Q - variance of process noise in the state transfer process: dx * dx 
    R - variance of measurement noise in the sensor: dz * dz
    x0 - initial state: dx * 1
    timesteps - total time step: 1 
    """
    # simulate groundtruth and get measurements
    sim = Simulation(
        x0=x0, 
        maneuver=action, 
        R=R, 
        Q=Q, 
        A = A,
        B = B,
        H = H)
    # create list of measurements
    xzs = [sim.transfer_and_sense() for _ in range(timesteps)]
    gx,zt = seperate_x_z(xzs)
    return gx,zt

def generate_ut01(A,B,H,Q,R,x0,timesteps,actions):
    # to generate action = 1 or 0 at time: t+1
    action = actions[0] # maneuver
    # simulate groundtruth and get measurements
    sim1 = Simulation(
        x0=x0, 
        maneuver=action, 
        R=R, 
        Q=Q, 
        A = A,
        B = B,
        H = H)
    
    xzs = [sim1.transfer_and_sense() for _ in range(timesteps)]
    gx,zt = seperate_x_z(xzs)    
    # u = 1 at last timestep
    new_action = actions[1]
    sim2 = Simulation(
        x0=gx[-2], 
        maneuver=new_action, 
        R=R, 
        Q=Q, 
        A = A,
        B = B,
        H = H)

    xzs_new_single = [sim2.transfer_and_sense() for _ in range(1)]
    gx_act1,zt_act1 = seperate_x_z(xzs_new_single)

    gx_act = []
    gx_act = gx[0:-1]
    zt_act = []
    zt_act = zt[0:-1]
    gx_act.append(gx_act1[0])
    zt_act.append(zt_act1[0])
    return gx,zt,gx_act,zt_act

def generate_ut_not0(A,B,H,Q,R,timesteps,action,gx,zt):
    sim_n = Simulation(
        x0=gx[-2], 
        maneuver=action, 
        R=R, 
        Q=Q, 
        A = A,
        B = B,
        H = H)
    xzs_new_single = [sim_n.transfer_and_sense() for _ in range(1)]
    gx_act1,zt_act1 = seperate_x_z(xzs_new_single)
    gx_act = []
    gx_act = gx[0:-1]
    zt_act = []
    zt_act = zt[0:-1]
    gx_act.append(gx_act1[0])
    zt_act.append(zt_act1[0])
    return gx_act,zt_act

def generate_ut(A,B,H,Q,R,x0,timesteps,actions):
    act_num = len(actions)
    if act_num == 2:
        gx,zt,gx_act,zt_act = generate_ut01(A,B,H,Q,R,x0,timesteps,actions)
        return gx,zt,gx_act,zt_act
    elif act_num == 3: 
        gx,zt = generate_fake(A,B,H,Q,R,x0,timesteps,actions[0])
        gx_act1,zt_act1 = generate_ut_not0(A,B,H,Q,R,timesteps,actions[1],gx,zt)
        gx_act2,zt_act2 = generate_ut_not0(A,B,H,Q,R,timesteps,actions[2],gx,zt)
        return gx,zt,gx_act1,zt_act1, gx_act2,zt_act2

def generate_ut_comb(A,B,H,Q,R,x0,timesteps,actions):
    """
    This function is to generate data when ut=actions=[0,1,2..]
    at the last timestep gien n-d ABHQR
    """
    act_num = len(actions)
    gxzts = []
    gx,zt = generate_fake(A,B,H,Q,R,x0,timesteps,actions[0])
    gxzts.append(gx)
    gxzts.append(zt)
    for i in range(1,act_num):        
        gx_act,zt_act = generate_ut_not0(A,B,H,Q,R,timesteps,actions[i],gx,zt)
        gxzts.append(gx_act)
        gxzts.append(zt_act)
    return gxzts    

def generate_sequential_ut(uts,ts):
    un = len(uts)
    ut_sq = []
    for i in range(un):
        t = ts[i]
        for j in range(t):
            ut_sq.append(uts[i])    
    return ut_sq        

def generate_seq_data(A,B,H,Q,R,x0,ut_sq):
    timesteps = len(ut_sq)
    gx = [] 
    zt = []
    sim_1 = Simulation(
        x0=x0, 
        maneuver=ut_sq[0], 
        R=R, 
        Q=Q, 
        A = A,
        B = B,
        H = H)
    for i in range(timesteps):
        if i==0:
            xzs_new = [sim_1.transfer_and_sense()]
            gxi,zti = seperate_x_z(xzs_new)
            gx.append(gxi[0])
            zt.append(zti[0])
        else:
            sim_n = Simulation(
                x0=gx[-1], 
                maneuver=ut_sq[i], 
                R=R, 
                Q=Q, 
                A = A,
                B = B,
                H = H)
            xzs_new = [sim_n.transfer_and_sense()]
            gxi,zti = seperate_x_z(xzs_new)
            gx.append(gxi[0])
            zt.append(zti[0])
    return gx,zt

""" Examples """    
def example_1d_generate_fake():
    A = np.array([1]).reshape(1, 1)
    B = np.array([[1]]).reshape(1, 1)
    H = np.array([1]).reshape(1, 1)
    Q = np.array([0.2 * 0.2]).reshape(1, 1)
    R = np.array([0.2 * 0.2]).reshape(1, 1)
    x0 = 0
    timesteps = 100
    action = 0
    gx,zt = generate_fake(A,B,H,Q,R,x0,timesteps,action)
    return gx,zt 

def example_Nd_generate_fake():
    A = np.array([[1,0],[0,1]]).reshape(2, 2)
    B = np.array([[1],[1]]).reshape(2, 1) 
    H = np.array([[1,0],[0,1]]).reshape(2, 2)
    Q = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(2, 2)
    R = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(2, 2)
    x0 = np.array([[0],[0]]).reshape(2, 1)
    timesteps = 100
    action = 0
    gx,zt = generate_fake(A,B,H,Q,R,x0,timesteps,action)
    return gx,zt

def example_generate_ut():
    A = np.array([[1,0],[0,1]]).reshape(2, 2)
    B = np.array([[1],[1]]).reshape(2, 1) 
    H = np.array([[1,0],[0,1]]).reshape(2, 2)
    Q = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(2, 2)
    R = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(2, 2)
    x0 = np.array([[0],[0]]).reshape(2, 1)
    timesteps = 100
    actions = [0,1,2]
    gxzts = generate_ut_comb(A,B,H,Q,R,x0,timesteps,actions)
    return gxzts

def example_generate_seq_ut():
    uts = [0,1]
    ts = [100,50]
    ut_sq = generate_sequential_ut(uts,ts)
    return ut_sq

def example_generate_data_seq_ut_1d():
    A = np.array([1]).reshape(1, 1)
    B = np.array([[1]]).reshape(1, 1)
    H = np.array([1]).reshape(1, 1)
    Q = np.array([0.2 * 0.2]).reshape(1, 1)
    R = np.array([0.2 * 0.2]).reshape(1, 1)
    x0 = 0
    uts = [0,1,2]
    ts = [100,50,50]
    ut_sq = generate_sequential_ut(uts,ts)
    gx,zt = generate_seq_data(A,B,H,Q,R,x0,ut_sq)
    return gx,zt

def example_generate_data_seq_ut_nd():
    dx = 2
    dz = 2
    """
    # 3 by 3
    A = np.array([[1,0,0],[0,1,0],[0,0,1]]).reshape(dx, dx)
    B = np.array([[1],[1],[1]]).reshape(dx, 1) 
    H = np.array([[1,0,0],[0,1,0],[0,0,1]]).reshape(dz, dx)
    Q = np.array([[0.2 * 0.2,0,0],[0,0.2*0.2,0],[0,0,0.2 * 0.2]]).reshape(dx, dx)
    R = np.array([[0.2 * 0.2,0,0],[0,0.2*0.2,0],[0,0,0.2 * 0.2]]).reshape(dz, dz)
    x0 = np.array([[0],[0],[0]]).reshape(dx, 1)
    """
    A = np.array([[1,0],[0,1]]).reshape(dx, dx)
    B = np.array([[1],[1]]).reshape(dx, 1) 
    H = np.array([[1,0],[0,1]]).reshape(dz, dx)
    Q = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(dx, dx)
    R = np.array([[0.2 * 0.2,0],[0,0.2 * 0.2]]).reshape(dz, dz)
    x0 = np.array([[0],[0]]).reshape(dx, 1)
    uts = [0,1,0,2]
    ts = [100,10,20,10]
    
    ut_sq = generate_sequential_ut(uts,ts)
    gx,zt = generate_seq_data(A,B,H,Q,R,x0,ut_sq)
    
    ground_truth = cnvdata.convert_array2list_nd(gx,dx)
    measurements = cnvdata.convert_array2list_nd(zt,dx)
    plotfgs.multiKf_plot(ground_truth,measurements)
    return gx,zt
    
if __name__ == '__main__':

    gx1,zt1 = example_generate_data_seq_ut_1d()    
    gx,zt = example_generate_data_seq_ut_nd()
