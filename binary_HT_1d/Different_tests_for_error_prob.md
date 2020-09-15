# Different tests for error probability program 

## Recall

$$
u_t=\left\{
\begin{aligned}
H_0: 0 & (𝑛𝑜 𝑒𝑣𝑒𝑛𝑡𝑠 ℎ𝑎𝑝𝑝𝑒𝑛 𝑐𝑎𝑠𝑒)  \\
H_1: 1 & (𝑒𝑣𝑒𝑛𝑡𝑠 ℎ𝑎𝑝𝑝𝑒𝑛 𝑐𝑎𝑠𝑒) \\
\end{aligned}
\right.
$$

* $Prob\{decide H_1 |H_1 true\} =P_D = ∫_λ^∝ N(HB∗1,σ^2)$

* $Prob\{decide H_1 |H_0 true\} =P_{FA} = ∫_λ^∝ N(HB∗0,σ^2)$

* $Prob\{decide H_0│H_1 true\}=P_M=1-P_D$

* $Prob\{decide H_0│H_0 true\}=P_{CR}=1-P_{FA}$

## Code implementation

![Figure_0](figs/Figure_0.png)

## Cases test

#### Case1:

![Figure_1](figs/Figure_11.png?resize=350)

![Figure_1](figs/Figure_1.png?resize=350)

#### Case2:

![Figure_1](figs/Figure_21.png?resize=350)

![Figure_1](figs/Figure_2.png?resize=350)

#### Case3:

![Figure_1](figs/Figure_31.png?resize=350)

![Figure_1](figs/Figure_3.png?resize=350)

#### Case4:

![Figure_1](figs/Figure_41.png?resize=350)

![Figure_1](figs/Figure_4.png?resize=350)

#### Case5:

![Figure_1](figs/Figure_51.png?resize=350)

![Figure_1](figs/Figure_5.png?resize=350)

#### Case6:

![Figure_1](figs/Figure_61.png?resize=350)

![Figure_1](figs/Figure_6.png?resize=350)

#### Case7:

![Figure_1](figs/Figure_71.png?resize=350)

![Figure_1](figs/Figure_7.png?resize=350)