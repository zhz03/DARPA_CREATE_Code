# README

Author: Zhaoliang Zheng and Zida Wu



## Downloading and Running the Repo

### System Requirement

To run all the code in this repository, your system need to meet the following requirements:

- Python 3.7
- [os](https://docs.python.org/3/library/os.html) library
- [sys](https://docs.python.org/3/library/sys.html) library
- [numpy](https://numpy.org/) library
- [scipy](https://www.scipy.org/scipylib/) library

### Prerequisite

Before you execute any code in the subfolder, you'll need to run '*addpath.py*' first. 

'*addpath.py*' will allow you to call functions module from different subfolder under this directory.

### Instruction

**1 There are 4 folders in the directory:**

- **Sim_KF_Pln_nd**: this folder contains n-d state simulation program, n-d state Kalman Filter program, n-d state sensing planning program and a comprehensive testing program that tests all these three.
- **binary_HT_1d**: this folder contains programs that can:
  - calculate the statistical error probabilities by using generated data
  - calculate theoretical error probabilities based on equations
  - Compare the above two error probabilities and do comprehensive test across different parameters space. 
- **binary_HT_nd**: this folder contains programs that can:
  - sampling from probability -nd algorithm could sample points
  - error probability calculation -nd algorithm could calculate the final n-d error probability
- **utility_functions**: this folder contains some utility functions that could help in data converting, plotting and visualization. 
- **E2Etest_1d**: this folder only contains four program, which can do end-to-end calculation (inputs $A,B,H,Q,R,x0,ts,uts$, outputs $Prob_D,Prob_{FA},Prob_M,Prob_{CR}$) as well as end-to-end comprehensive validation and testing. The **Sensor_planner** program can be directly integrated with other groups. 

In each folder, there is another README.md to explain more details about each program. And in each program, there are comments and example functions to show how to use and run the code.

**3** To see the result of comprehensive test on 1d state binary hypothesis testing, please go to *binary_HT_1d* folder and run *Gd_comp.py* program.

**4** To see the results of comprehensive test on n-d state binary hypothesis testing, please go to *binary_HT_nd* folder and run *bin_HT_nd_verification.py* *Error_prob_cal_verfication.py*   

**5** To see the result of comprehensive test on nd state simulation, nd state kalman filter as well as nd state planning, please go to *Sim_KF_Pln_nd* folder to run *CompTest_diff_sysM.py* program

### End-to-End Test

- The algorithm can be divided into two parts. One is estimator branch based on statistical estimation of ut. Another is planner branch which based on theoratical estimation.
- The entrance of estimator is Simulations/Simulation.py; The entrance of planner is Sensor_planner/Sensor_planner.py.
- For convenience, we designed an end-to-end function in E2E_test/E2E_validation.py. You can adjust params of the system, such as mode(1d/nd), iteration num, nHy and etc.. After that,  just run this script.
- The output of the end-to-end test is the probability of detection comparison between simulation (statistics) and theoretical value.