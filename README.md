# README

Author: Zhaoliang Zheng



## Downloading and Running the Repo

### System Requirement

To run all the code in this repository, your system need to meet the following requirements:

- Python 3.7
- [os](https://docs.python.org/3/library/os.html) library
- [sys](https://docs.python.org/3/library/sys.html) library
- [numpy](https://numpy.org/) library
- [scipy](https://www.scipy.org/scipylib/) library
- copy

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
- **utility_functions**: this folder contains some utility functions that could help in data converting, plotting and visualization. 

- **E2Etest_1d**: this folder only contains four program, which can do end-to-end calculation (inputs $A,B,H,Q,R,x0,ts,uts$, outputs $Prob_D,Prob_{FA},Prob_M,Prob_{CR}$) as well as end-to-end comprehensive validation and testing. The **Sensor_planner** program can be directly integrated with other groups. 

In each folder, there is another README.md to explain more details about each program. And in each program, there are comments and example functions to show how to use and run the code.

**2 To see the end-to-end results, please go to *E2Etest_1d* folder and run the *E2Etest* program.** 

**3 To see the result of comprehensive test on 1d state binary hypothesis testing, please go to *binary_HT_1d* folder and run *Gd_comp.py* program.**

**4 To see the result of comprehensive test on nd state simulation, nd state kalman filter as well as nd state planning, please go to *Sim_KF_Pln_nd* folder to run *CompTest_diff_sysM.py* program**

