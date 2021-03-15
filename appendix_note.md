### Important terms/variables in code:

binary_1d: u (event) is binary values: 0/1, 1d means the z is 1 dimention (or x contains 1 sensor)

binary_nd: nd means z is n dimentions (or x contains n sensors)

multi_nd: u (event) has multi hypothesis and z is n dimentions (or x contains n sensors)

In code, multi-hypothesis of u event means there are multiple distributions list(miu_i, sigma_i), n-d means the dimention of u is n: u.ndim=(n,)

In system_setup_generator section, there are variables that ts/uts/ut/T/nHy. In practice, *nHy*is the multi hypothesis of u, namely *ut*=np.arange(nHy). While we want to simulate our system iteration multi times, the sequence of u is uts, *uts* = random.randint(0, nHy, nHy). In each item of uts, we iterate *ts* times. T = sum(ts).


   ### Quick notes of Structure of the code:

    - The algorithm can be divided into two parts. One is estimator branch based on statistical estimation of ut. Another is planner branch which based on theoratical estimation.
    
    - The entrance of estimator is Simulations/Simulation.py; The entrance of planner is Sensor_planner/Sensor_planner.py.
    
    - In this code, we generate numbers of System Model (SM) and each SM with numbers of trials. In each trials, we run the SM for T timestep and use the sequence of data of the T period as our test data.
    
    - All system model (SM) parameters are set in E2Etest/System_setup_generator.py. (nHy = multiple hypothesis；ts=[T, n])
    
    - All system models were generated in E2Etest/SM_generator_nd.py [ABHQR of SM)

   ### Something hasn't been finished:

    1. [Fixed] In etimator part (correponding to Simulations file):
       Currently, it is only binary_1d esimator. To make it into binary_nd, we need to modify two functions: Estimator.Decision_making and Estimator.Bayesian_analysis; To turn it into multi_nd, we need to change SM_generator_1d and modify the above two functions.
    2. [Fixed] In sensor planner part (corresponding to Sensor_planner file):
       Currently, Sensor_planner.py lacks the last multi-hypothesis-test function.
    3. [Fixed] End-to-end test:
       Currently, we almost finished the code of estimator (namely simulation) and sensor planner. But we need to realize the end-to-end test to combine them into a whole.
    4. [Fixed] Sampling based Bayesian decision makeing in Sensor Planner:
       Calculation of Prob_D is wrong: Bin_stat_hyp_test_1d
   ### Quick FAQ:

      1. What is the final dimension of probability of detection? -> If nHy means the possible event of u. The final dimensions of probability detection matrix is (nHy, nHy). 
      2. What is the practical dimension of event u? ->du =1 
      3. Why in simulator.py there are a scalling value 100000? ->Iteration of ABHQR would make value of mean/cov quickly increasing. When the mean/cov  matrix value is more than 100000, scale down the matrix to avoid numerical overflow.
      4. What the meaning of T? ->To simulate multi-iteration before the final estimation. We only concern about the last two value of T iterations.
      5. What is the difference between Error_prob_cal_oldV and Error_pro_cal in  binary_HT_nd/Error_prob_cal.py? ->Error_prob_cal_oldV is the method to find the intersection lamda; Error_pro_cal  is based on sampling to find the intersection point.
