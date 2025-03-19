(work in progress)

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

Now I try to simulate the 6 tanks in series process exactly with a **First-Order-Dead-Time (FODT)** process, also often called First-Order Plus Dead-Time (FOPDT) process, based on the estimated process time constants of Part 2, here without simulated process noise: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%203%3A%20first-order-dead-time%20(FODT)/3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process.py

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













