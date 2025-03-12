2025-03-12

Now let's combine the current achievements and apply them to the unit step response of a process with lag of 6th order: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f6a_6xPT1_step_response_parameter_estimation_for_lambda_tuning.py

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/3.5f6a_6xPT1_step_response_parameter_estimation_for_lambda_tuning.png)

Works like a charm so far, even with the simple Euler forward approximation.

------

2025-03-10

So far the simulation of noise on the measurement, which goes into the controller algorithm, was presented with an extra variable *x1_noisy* in program https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/step_response_parameter_estimation_for_lambda_tuning%20-%20experiments.py, a luxury we don't have in the closed control loop.

Now I got the idea to take back older and memorized noise data points from system state variable x1 when updating *x1[k+1]*:

```
for k in range(2,STEPS):
    ...
    if k < STEPS-1:
        ...
        x1[k+1] = 1.0/a1**3.0 * ((a1**3.0 - A0)*x1[k-2]/x1_noise[k-2] \
                                     - 3.0*a1**2.0*(a0*h - a1)*x1[k]/x1_noise[k] \
                                     - 3.0*a1*A1*x1[k-1]/x1_noise[k-1] \
                                     + b0*h**3.0*u[k-2])
        ...
        if NOISE is True:
            x1[k+1] = x1[k+1] * x1_noise[k+1]
```

from: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/step_response_parameter_estimation_for_lambda_tuning%20-%20experiments2.py

So instead of *x1[k-2]* the term *x1[k-2]/x1_noise[k-2]* is being employed in the difference equation for example, which apparently produces the same noisy signal like we have seen so far:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20-%20experiments2%20b.png)

And remember that the controller algorithm is being executed *before* the process simulation part in the system simulation loop: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f3_PI-control-loop_with_3xPT1_Euler_forward_process.py

Then I double checked this noise solution on *x1* with the noiseless system state variable *x1_clean*:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20-%20experiments2%20c.png)

This looks OK.

Let's see how this works in an updated, closed control loop program.

------

2025-03-08c

Well, nothing keeps us from applying a Savitzky–Golay filter two times:

```
if min(x1_grad) < -0.00001:  # leave some room for error
    NOISE_PRESENT = True

    # Savitzky-Golay (SG) filtering of measurement:
    WINDOW_SIZE = 41  # 71: manual tuning --> OKish for one filter run
    POLYNOMIAL_ORDER = 1  # 1: manual tuning --> better tuning rule??

    # 1st filter run:
    x1_hat = savgol_filter(x1_org, WINDOW_SIZE, POLYNOMIAL_ORDER)

    # 2nd filter run:
    x1_hat2 = savgol_filter(x1_hat, WINDOW_SIZE, POLYNOMIAL_ORDER)

```

...and Whoa! After some short experimentation a (non-sluggish) window size of 41 with double filtering looks quite convincing:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2006j.png)

Here a double filtered step response:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2006k.png)

With the exception of the (usual) filter artefact (of a Savitzky–Golay filter) at the beginning of the step response, both gradient curves are undistinguishable now:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2006l.png)

..which makes finding a "good" tangential line much easier:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2006m.png)

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2006n.png)

And so the estimated times should not be too far off from the true but uknown values:

*maximum gradient of potentially noisy measurement x1 = 0.00272*

*at time = 2.18000*

*maximum gradient of noise-free x1 = 0.00272*

*at time = 2.01000*

*SIGMA = 0.01*

*Savitzky-Golay filtering (2x):*

*polynomial order = 1 (manually tuned)*

*window size = 41 (manually tuned)*

------

2025-03-08b

A natural idea to estimate a plausible time for the maximum slope is to filter the noisy measurement.

By the way: the current and very simple idea to detect a noisy measurement is only working for positive step responses of self-regulating processes (https://www.isa.org/intech-home/2016/may-june/departments/loop-tuning-basics-self-regulating-processes), where - without noise - the gradient must be zero or higher:

```
if min(x1_grad) < -0.00001:  # leave some room for error
    NOISE_PRESENT = True
    ...
```

Since we don't have to filter a noisy measurement while the process is developing (with an online solution like a moving average for example), we can do it *after* the bump test, here with a Savitzky–Golay filter: https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter

For example like this:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2006a.png)

However, after some experimentations it shows that finding "good" filter parameters, that is window size and polynomial order, isn't so straightforward.

Just increasing the window size and/or the polynomial order doesn't lead to a "good" solution apparently:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2006g.png)

So, I've written a test routine to check manually what at least the best window size good be by "visual inspection" (a polynomial order of just 1 seems to be a good fit though):


```
ws = np.arange(11,161,10)  # test a range of window sizes
x1_test_time_ws =  [0.0 for i in ws]
WS_TESTS = 10000
time_i = 0
for ws_i in ws:
    x1_test_time = []
    for i in range(0,WS_TESTS):
        x1_sg = savgol_filter(x1_org, ws_i, 1)  # polynomial order seems to be OK
        x1_grad_t = np.gradient(x1_sg)
        max_x1_grad_index_t = np.argmax(x1_grad_t)
        max_x1_grad_time_t = t[max_x1_grad_index_t]
        x1_test_time.append(max_x1_grad_time_t)

    x1_test_time_ws[time_i] = np.mean(x1_test_time)
    time_i += 1

x1_test_time_min = np.empty(len(x1_test_time_ws))
x1_test_time_min = abs(x1_test_time_ws - max_x1_grad_no_noise_time)
good_ws = ws[np.argmin(x1_test_time_min)]

print(f'\nA good window size for a Savitzky-Golay filter\
 based on {WS_TESTS} test runs is {good_ws}\n')
```

...and as documented in the comment of the test routine (https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/step_response_parameter_estimation_for_lambda_tuning%20-%20experiments.py) this test only showed that moving the window size up and down (with the given noise) doesn't find a convincing solution.

<br/>

What now?

------

2025-03-08a

However, there's one big drawback with a simulation like this:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2000.png)

..and it's noise. Nothing in nature runs so smoothly because nature is full of noise:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2004.png)

By the way: I changed the noise logic completely since the old solution just leads to a nonlinear system:

```
for k in range(2,STEPS):
    ...
    # process:
    if k < STEPS-1:
        ...
        # x1_KJA[k+1] = x1_KJA[k+1] * x1_KJA_noise[k+1]
```

from: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f3_PI-control-loop_with_3xPT1_Euler_forward_process.py

The "clean" state variable for the process simulation must be kept separated from a noisy measurement, where noise, be it additive or multiplicative, is **not injected** into a state variable, which has  memory, but is just applied to it at an extra array:

```
x1       = np.zeros([STEPS])  # clean measurement = clean process output
x1_noisy = np.zeros([STEPS])  # noisy measurement = noisy process output
# noise has no memory (--> Markov process) and thus the simulation of a noisy measurement
# needs an extra array
...
# process simulation loop:
for k in range(2,STEPS):

    # process:
    if k < STEPS-1:
        ...
        x1[k+1] = 1.0/a1**3.0 * ((a1**3.0 - A0)*x1[k-2] \
                                     - 3.0*a1**2.0*(a0*h - a1)*x1[k] \
                                     - 3.0*a1*A1*x1[k-1]
                                     + b0*h**3.0*u[k-2])

        if NOISE is True:
            x1_noisy[k+1] = x1[k+1] * x1_noise[k+1]
...
```

<br/>

And the ubiquitous presence of noise is a big problem when calculating the gradient, to get the maximum slope, in a naive way:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2005.png)

*NOISE_PRESENT = True*

*maximum gradient of x1 = 0.02217*

*at time = 6.15000*

=> the targeted maximum slope for the tangential line is for sure not to be found at 6.15 seconds.

------

2025-03-07:

Re: *I'm estimating these parameter values at:*

*dead time td ≈ 0.8 seconds*

*time constant tau ≈ 2.4 seconds*

A natural question arises here: can the computer estimate these process parameters conveniently and also more accurately?

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2000.png)

Of course, the program can be expanded for this job:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2001.png)

My idea here is simply to first compute the gradient of the step response d(x1(t))/dt (in blue color above): see from the link below:

*Find the maximum slope of the PV response curve. This will be at the point of inflection. Draw a line tangential through the PV response curve at this point.*

```
# calculating the gradient of the measurement:
x1_grad = np.gradient(x1)
```

To make the really small values of the gradient more visible (*np.max(x1_grad) = 0.002720*) in the same diagram I applied a factor of 20 to the gradient curve.

So, I got this tangential line in black color at the step response:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2002.png)

See again from the link below:

*Dead Time (td) ... Extend this line to intersect with the original level of the PV before the step in CO. Take note of the time value at this intersection.*

*td = time difference between the change in CO and the intersection of the tangential line and the original PV level*

...

*Time Constant (tau) ... tau = time difference between intersection at the end of dead time, and the PV reaching 63% of its total change*

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/pictures/step_response_parameter_estimation_for_lambda_tuning%20--%2003.png)

So, this algorithm estimated these parameter values:

* dead time td ≈ 0.83 seconds

* time constant tau ≈ 2.31 seconds

..which don't seem to too far off from my manual readings.

That was not too complicated to program.

------

2025-03-06:

Now let's put another tank, as a PT1 model again, in series. Then the unit step responses so far look like this:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f3_PI-control-loop_with_3xPT1_Euler_forward_process%20-%20step%20responses%20PT1%2C%20PT2%2C%20PT3.png)

..and have been generated with this program: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f3_PI-control-loop_with_3xPT1_Euler_forward_process.py

(this program, at least for now, contains a lot of commented source code lines for all the experimentations)

<br/>

With three tanks in series I can manually and roughly make parameter estimations according to the lambda tuning rules; see from here again: https://blog.opticontrols.com/archives/260

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/Step%20Test%20for%20Lambda%20Tuning.png)

I'm estimating these parameter values at:

* dead time td ≈ 0.8 seconds
* time constant tau ≈ 2.4 seconds

To not have a too sluggish step response under feedback control, I've chosen a "taucl factor" of 1.5: 

*Generally, the value for taucl should be set between one and three times the value of tau.*

```
tau=2.4  # [sec]; rough human reading
gp=1.0
taucl_factor = 1.5  # org: 3.0
taucl=taucl_factor*tau
td=0.8  # [sec]; rough human reading
# =>
Kp = tau/(gp * (taucl + td))
Tn = tau
```

Finally, the step response of the closed control loop with three tanks in series and the chosen control parameters looks like this:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20Part%202%3A%20first-order-dead-time%20(FODT)/3.5f3_PI-control-loop_with_3xPT1_Euler_forward_process.png)


There's a slight modification with this version of the simulation program. It's the starting at index 2, not 1 like before:

```
for k in range(2,STEPS):
    ...
```

By this offset, the program is not reading "values from the past" at the process simulation part:

```
    # process:
    if k < STEPS-1:
        ...
        x1_KJA[k+1] = 1.0/a1**3.0 * ((a1**3.0 - A0)*x1_KJA[k-2] \
                                     - 3.0*a1**2.0*(a0*h - a1)*x1_KJA[k] \
                                     - 3.0*a1*A1*x1_KJA[k-1]
                                     + b0*h**3.0*u_KJA[k-2])
```


------

2025-03-05c:

By default the SymPy Live Shell produces an output in form of a LaTeX expression, something which may be of limited use for further human interaction to get the final difference equation for the source code.

So, I expanded the little SymPy script to print the result in plain text format:

```
a1, a0, b0, X1, U, h, s = symbols('a1, a0, b0, X1, U, h, s')
s =  (z - 1 ) / h
X1 = b0 / ((a1*s + a0)*(a1*s + a0)) * U
result = simplify(collect(expand(X1),z))
str(result)
```

In case of the two PT1 terms in series the output in plain text format looks like this:

```
'U*b0*h**2/(a0**2*h**2 - 2*a0*a1*h + a1**2*(z**2 + 1) + 2*a1*z*(a0*h - a1))'
```



------

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

