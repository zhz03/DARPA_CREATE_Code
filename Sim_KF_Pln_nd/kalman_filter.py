"""
Created on Wed Mar  4 13:53:32 2020

@author: Zhaoliang
"""

import numpy as np
import matplotlib.pyplot as plt
import Sim_KF_Pln_nd.simulation as sim1
import utility_functions.plot_figures as plotfgs
import utility_functions.convert_data as cnvdata
import utility_functions.CompP2SHist as CompP2SHist

class KalmanFilter(object):
    def __init__(self, A = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(A is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = A.shape[1]
        self.m = H.shape[1]
        self.A = A
        self.H = H
        self.B = np.zeros((self.n, 1)) if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.m) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = self.P - np.dot(np.dot(K,self.H),self.P)
        return self.P,self.x
      
def KFprocess(A,B,H,Q,R,measurements):
    
    kf = KalmanFilter(A=A,B=B,H=H,Q=Q,R=R)
    
    predictions = []
    estimates = []
    sigma = []
    for z in measurements:
        predictions.append(kf.predict())
        P,x=kf.update(z)
        estimates.append(x)
        sigma.append(P)

    return predictions,sigma,estimates

"""
Examples
"""
def example_1d():
    dx = 1
    dz = 1
    A = np.array([1]).reshape(dx, dx)
    B = np.array([1]).reshape(dx, 1) 
    H = np.array([1]).reshape(dz, dx)
    Q = np.array([0.2*0.2]).reshape(dx, dx)
    R = np.array([0.2*0.2]).reshape(dz, dz)
    x0 = np.array([0]).reshape(dx, 1)    
    uts = [0]
    ts = [1000]
    
    ut_sq = sim1.generate_sequential_ut(uts,ts)
    ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq)
    predict,sigma,estimates = KFprocess(A,B,H,Q,R,measurements)
    ground_truth = cnvdata.convert_array2list_nd(ground_truth,dx)
    measurements = cnvdata.convert_array2list_nd(measurements,dz)
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    predict = cnvdata.convert_array2list_nd(predict,dz)
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
    error = [[ground_truth[i][j] - estimates[i][j] for j in range(ts[0])] for i in range(len(ground_truth))]    

    plotfgs.KF_plot(measurements[0],ground_truth[0],estimates[0],sigma[-1][0],simulation=True)
    CompP2S = CompP2SHist.Compare_pln2statis_hist(mean_pln = uts,Sigma_pln = sigma[-1][0],name='plan')
    mean_error,var_error = CompP2S.numer_compare(error[0])
    CompP2S.visualization_compare(error[0])

def example_2d():
    dx = 2
    dz = 2
    q = np.array([[0.2,0.],[0.,0.2]]).reshape(dx, dx)
    r = np.array([[0.2,0],[0,0.2]]).reshape(dz, dz)
    A = np.array([[1,0],[0,1]]).reshape(dx, dx)
    B = np.array([[1],[1]]).reshape(dx, 1) 
    H = np.array([[1,0],[0,1]]).reshape(dz, dx)
    Q = np.dot(q,q.T)
    R = np.dot(r,r.T)
    x0 = np.array([[0],[0]]).reshape(dx, 1)
    uts = [0]
    ts = [1000]
    
    ut_sq = sim1.generate_sequential_ut(uts,ts)
    ground_truth,measurements = sim1.generate_seq_data(A,B,H,Q,R,x0,ut_sq)
    predict,sigma,estimates = KFprocess(A,B,H,Q,R,measurements)
    
    ground_truth = cnvdata.convert_array2list_nd(ground_truth,dx)
    measurements = cnvdata.convert_array2list_nd(measurements,dz)
    estimates = cnvdata.convert_array2list_nd(estimates,dx)
    predict = cnvdata.convert_array2list_nd(predict,dz)
    plotfgs.multiKf_plot(measurements,ground_truth,estimates,kfest_flag = True)
if __name__ == '__main__':
    example_1d()
    