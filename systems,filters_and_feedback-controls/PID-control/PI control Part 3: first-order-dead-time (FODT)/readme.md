2025-03-18

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













