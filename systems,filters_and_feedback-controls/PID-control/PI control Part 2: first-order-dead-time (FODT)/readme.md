2025-03-05:

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

