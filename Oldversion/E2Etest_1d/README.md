The CREATE-SP-flowchart.pdf illustrate the workflow of this E2E-1d folder.

There are several core functions in this folder.  

## Core functions

### 1 Simulation_validation 

- This function takes $A,B,H,Q,R,x_0,uts,ts,sam_{numb}$ as inputs.

Where,

 $A,B,H,Q,R$ are system matrices,

$x_0$ is initial state

$uts$ is system inputs

$ts$ is time duration for inputs

$sam_{numb}$ is the sample numbers = 1000

- This function outputs statistical error probabilities: $Prob_D^{stats}$, $Prob_{FA}^{stats}$, $Prob_M^{stats}$, $Prob_{CR}^{stats}$ 

### 2 Sensor_planner (This can be directly integrated with other groups)

- This function takes $A,B,H,Q,R,uts,ts$ as inputs.
- This function outputs theoretical error probabilities:  $Prob_D,Prob_{FA},Prob_M,Prob_{CR}$ 

### 3 E2E_validation

- This function takes $A,B,H,Q,R,x_0,uts,ts,sam_{numb}$ as inputs.

- Compare the difference between statistical error probabilities and theoretical error probabilities in order to verify this End-to-End pipeline: 

$$
error_{D,FA,M,CR} = Prob_{D,FA,M,CR}^{stats} - Prob_{D,FA,M,CR}
$$

