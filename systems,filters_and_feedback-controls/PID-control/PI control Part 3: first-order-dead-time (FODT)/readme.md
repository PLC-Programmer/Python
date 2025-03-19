(work in progress)

<br/>

2025-03-19a

Back in January I implemented chapter "4.1  Simulation  der  Zweipunktregelung  einer  Regelstrecke  mit  Totzeit" / "4.1. Simulation of the on-off control of a process with dead time" from the Ernst-Guenther Feindt book: "Computersimulation von Regelungen - Modellbildung und Softwareentwicklung": https://www.degruyter.com/document/doi/10.1515/9783486799002-005/html

See my source code from here: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/4.1_two-point_control_with_dead_time.py

There the dead time of the process is implemented like this:

```
# system simulation loop:
for k in range(1,STEPS):

    # controller algo:
    e1  = w_1 - x1[k-1]
    if e1 > 0:
        u_ = 1.0
    else:
        u_ = -1.0

    # deadtime simulation loop:
    u[0] = u_
    for m in range(STEPS_DT-1,0,-1):
        u[m] = u[m-1]
    y = u[-1]  # output of deadtime process block

    ...
```

Be aware that the time continuous part of this process is a IT1 term, so an integrating term with a lag of first order. Therefore two differential equations (f1, f2) are to be integrated.


To achieve my goal as stated below, I'm now trying to merge this algorithm with the other requirements of the simulated FODT process within a state-space solution with PI control.




------

2025-03-18b

Something is still missing to bring the puzzle together:

* on hand side we have the original **state-space solution for the PI control** of an undamped PT2 process in program "3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space.py": https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space.py with this result:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space%20-%20a.png)

See the state-space solution starting there at comment "# compare with the original state space solution from book's ch.3.5:".

* ..and on the other hand side the recent **non-state-space solution with the Karl Johan Åström-PI controller for a FODT process** to emulate and control a lag process of 6th order ("6 x PT1") with program "3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process.py": https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process.py

<br/>

Remember that "My final goal is to have a real PI(D) controller which can control a first order plus dead time process (FODT) with acceptable results" from: [https://github.com/PLC-Programmer/Python/tree/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process](https://github.com/PLC-Programmer/Python/blob/master/systems,filters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/readme.md#my-final-goal)

I guess with program "3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process.py" this is partly accomplished, though only a PI controller is in action there (remember: "Although Lambda / IMC tuning rules have also been derived for PID controllers, there is little point in using derivative control in a Lambda-tuned controller." from https://blog.opticontrols.com/archives/260).

However, I would like to compare the solution of the 3.5g1-program with a state-space solution with PI controller for **the same FODT process**.

So far I haven't presented a state-space solution for a process with dead time.

This is to be done yet.


------

#### 2025-03-18a

This is the continuation from Part 2: https://github.com/PLC-Programmer/Python/tree/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)

Now I try to simulate the 6 tanks in series process exactly with a **First-Order-Dead-Time (FODT)** process, also often called First-Order Plus Dead-Time (FOPDT) process, based on the estimated process time constants of Part 2.

2025-03-19: the first version was too fast.

The coefficients of the polynomial of the transfer function of the time continuous PT1 term inside the FODT term are not the same, at least partly not the same, as the coefficients of the polynomial of the transfer function of the PT1 basic block from the 6xPT1 process: 

```
# PT1 process coefficients (part of FODT):
# numerator polynomial:   b0
# denominator polynomial: (a1*s + a0)
# 2025-03-19: with K = 1.0, a0 = 1.0 => a1 = tau = 3.57, see from below
b0 = 1.0
a0 = 1.0
a1 = 3.57


# 6 x PT1 process coefficients:
b0_org = 1.0
a0_org = 1.0
a1_org = 1.0
# numerator polynomial:   b0_org
# denominator polynomial: (a1_org*s + a0_org)^6
```

From: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process.py

So at first, I compare both step responses to see if the FODT simulation is a suitable emulation of the original 6xPT1 process: 

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/pictures/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process%200%20-%20bump%20test.png)

The difference area between the two bump test curves up to 25 seconds is about -39.7, which means that the original 6xPT1 step response curve covers more area up to 25 seconds. This represents about 2.1% more area than the area of the blue FODT emulation process.

Unit step response of the closed control loop:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/pictures/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process%20a.png)

Here I added another set point jump at second 50:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/pictures/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process%20b.png)

Here the same with simulated process noise:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/pictures/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process%20c.png)

Now the simulation of a transient disturbance:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/pictures/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process%20d.png)

Be aware that the disturbance amplitude above has the bold value of 0.01 compared to the minuscule value of 0.000000001 in Part 2.

This stark difference stems from the now fundamentally changed process simulation with the FODT approximation implemented like this:

```
        # process:
        # Euler forward emulation of
        # process transfer function G(s) = b0 / ((a0 + a1*s)
        ...
        x1[k+1] = 1/a1 * (b0*h*u_KJA[k-DELAY]
                          + (a1 - a0*h)*x1[k]/x1_noise[k])
```

Now the process dead time is directly considered at controller output *u_KJA[k]*, which became *u_KJA[k-DELAY]*, with *DELAY = int(td/h)* time steps of dead time which just now goes directly into our usual difference equation of a PT1 term with an Euler forward emulation.

Here the same with a disturbance amplitude of only 0.001:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/pictures/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process%20e.png)

..something which looks more realistic.
