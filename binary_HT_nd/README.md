# Binary_HT_nd README

### Overview 

This folder contains three main programs:

- bin_HT_nd
- sampling_binGDnd
- Error_prob_cal

The flowchart of these program is shown as follow:

![](figs/flowchart.png)

### Sampling_binGDnd 

This program is to sample points from probability distribution for n-dimensional space. 

There are three sub-programs about it:

- sampling_binGDnd

- sampling_binGDnd_verification

  - In the **sampling_binGDnd_verification** program, you can run different numbers of trials by changing the *trial_num* parameters. 
  - The generated data will be saved in the "*data_storage*" folder once the **sampling_binGDnd_verification** program finish running 

- sampling_binGDnd_Plot

  - In the **sampling_binGDnd_Plot** program, we can load the saved data into the program and decide which results of trials that we want to plot. 

  - The plotting results will be saved in the *figs* folder. 

  - The statistical comparison results from $Pr_D,Pr_M,Pr_FA,Pr_CR$

    ![](figs/stat_1d/Error_mean0.jpg)

    ![](figs/stat_1d/Error_mean1.jpg)

    ![](figs/stat_1d/Error_var0.jpg)

    ![](figs/stat_1d/Error_var1.jpg)

### Error_prob_cal

This program is to calculate the error probabilities based on sample points.

There are two sub-programs about it:

- Error_prob_cal

  ![](figs/sub-system-flowchart.png)

- Error_prob_cal_verification

  - In this program, we could verify 2d error probabilities 

  - It can also save the figures what we select or all the figures into the *figs/error_prob_verification_2d* folder

  - Some results:

    ![](figs/error_prob_verification_2d/0.jpg)

    ![](figs/error_prob_verification_2d/10.jpg)
    
    ![](figs/error_prob_verification_2d/30.jpg)

### Verification

![](figs/verification.png)

*Numerical and statistical analysis -nd* function is to verify and calculate  the norm of the error of the mean and variance of the different dimensions.

The inputs of this function are: $Error_{i}^{mean} \in \mathcal{R}^{1 \times n}, Error_{i}^{var} \in \mathcal{R}^{n \times n}$ 
$$
Error_{i}^{mean} = mean_{i}^{sample} - mean_{i}^{truth}, i \in \mathcal{R}_{trials}^{n}
$$

$$
Error_{i}^{var} = Var_{i}^{sample} - Var_{i}^{truth}, i \in \mathcal{R}_{trials}^{n}
$$

The outputs of this function are: $||Error_{i}^{mean}||_2\in \mathcal{R}^{n},||Error_{i}^{var}||_2\in \mathcal{R}^{n}$

The outputs plots for 2-d:

![](figs/stat_2d/Mean0_norms.jpg) ![](figs/stat_2d/Mean1_norms.jpg)

![](figs/stat_2d/Var0_norms.jpg)![](figs/stat_2d/Var1_norms.jpg)