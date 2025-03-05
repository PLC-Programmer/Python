2025-03-05b:

Now let's put two tanks, each with a PT1 model, in series. Transfer function of this process will look like this:

X1(s)/U(s) = b0 / ((a1·s + a0)·(a1·s + a0))

Again I'm using the Euler forward emulation of this process with its approximation s ≈  (z - 1 ) / h

The resulting (and approximated) transfer function starts to become complicated. So, I'm using the SymPy Live Shell for helping with the indermediate steps: https://live.sympy.org/

```
a1, a0, b0, X1, U, h, s = symbols('a1, a0, b0, X1, U, h, s')
s =  (z - 1 ) / h
X1 = b0 / ((a1*s + a0)*(a1*s + a0)) * U
simplify(collect(expand(X1),z))
```

Everything else unchanged, including the controller parameters (https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f2_PI-control-loop_with_2xPT1_Euler_forward_process.py) the step response looks promising:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f2_PI-control-loop_with_2xPT1_Euler_forward_process.png)

 
I can't call this done according to "(lambda tuning rules)" anymore, so I left this comment away.

Anyhow, the step response became faster, and with a very slight overshoot, which is logical with unchanged controller parameters (which are a little bit too fast now).


------

2025-03-05a:

I start with only one tank as my process, that is a PT1 term (lag of first order): https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f1_PI-control-loop_with_PT1_Euler_forward_process.py

The unit step response will look like this:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f1_PI-control-loop_with_PT1_Euler_forward_process.png)

As announced below, I used the Lambda Tuning Rules (https://blog.opticontrols.com/archives/260) with PI controller parameters:

- Controller Gain Kc = tau/(gp x (taucl + td))
- Integral Time Ti = tau

```
tau=1.0
gp=1.0
taucl=3*tau
td=0.0
# =>
Kp = tau/(gp * (taucl + td))
Tn = tau
```



------
2025-02-26:

First-order-dead-time (FODT) is also called first-order-plus-dead-time, FOPDT:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/Step%20response%20for%20tanks%20in%20series.png)

From: Process Dynamics, Operations, and Control, Lesson 7, High Order Overdamped Processes, 2006: https://ocw.mit.edu/courses/10-450-process-dynamics-operations-and-control-spring-2006/9323048e3f2ffc52e50708760833e598_7_high_order.pdf

<br/>

Otherwise:

There's a ton of (academic) literature on feedback control of FODT processes out there -- but boy -- just a simple overview of tuning rules for this ubiquitous combination of a PI controller and a PT1 (lag of first order) process?!?

So, for the first experiment I apply the **Lambda Tuning Rules** (https://blog.opticontrols.com/archives/260) as a starting point.

