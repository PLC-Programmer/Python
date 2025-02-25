2025-02-25: work in progress

The German acronym "PT2" (https://de.wikipedia.org/wiki/PT2-Glied) is equivalent to a "2nd order lag" process.

\
In this directory is refer to this old but excellent book from 1999 by Ernst-Guenther Feindt: **"Computersimulation von Regelungen - 
Modellbildung und Softwareentwicklung"** (https://www.degruyter.com/document/doi/10.1515/9783486799002/html) and there specifically to chapter "3.5 Simulation einer kontinuierlichen Regelung, Kontrolle der Simulationsgenauigkeit, Parameterbestimmung mit Simulation einer selbstoptimierenden Regelung" ("Simulation of continuous control, control of simulation accuracy, parameter determination with simulation of self-optimizing control") under section "3. Simulation der kontinuierlichen Regelungen" ("Simulation of continuous controls").

Actually, this book is the source of my motivation for the whole "PID control" directory: https://github.com/PLC-Programmer/Python/tree/master/systems%2Cfilters_and_feedback-controls/PID-control

Ironically this book does not feature a PID controller, all examples are only done with PI controllers.

<br/>

The featured process can be modeled with transfer function **G(s) = b0 / (s² + a1·s + a0)**

with these parameters:

* b0 = 1.3
* a0 = 1.0
* a1 = 1.6

which leads to a naturally undamped process with parameters:

* ω0 = omega0 = np.sqrt(a0) (natural angular frequency) => ω0 = 1.0/sec
* a1 = 2·D·ω0
* D = a1 / 2.0 / omega0 = 0.8, which makes this process underdamped (D < 1.0)

<br/>

My final goal in this directory is to have a real PID controller which can control a **first order plus dead time process (FODT)** with acceptable results:

https://towardsai.net/p/artificial-intelligence/classic-methods-for-identification-of-first-order-plus-dead-time-fopdt-systems

<br/>

While trying so and for now, I'm implementing both parts, controller + process, with non-state-space solutions; see from here why:

https://github.com/PLC-Programmer/Python/blob/master/systems,filters_and_feedback-controls/PID-control/readme.md#implementation-of-a-pid-controller-as-a-state-space-controller

<br/>

### 1/ First exercise: How to generally model a 2nd order lag process (without state-space representation)? -- Approximations of continuous time differential equations

I refer to this excellent paper from 2013 by Prof Lino Guzzella: **"Discrete Time Control Systems"**:

https://idsc.ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/Digital-Control-Systems/Slides_DigReg_2013.pdf

https://idsc.ethz.ch/the-institute/people/person-detail.MjgwNzc=.TGlzdC82MjMsMjA4ODk3NDEzOQ==.html

<br/>

Principially, there are three possibilities to **approximate** a continuous time system (like our use case G(s) = b0 / (s² + a1·s + a0)) into a discrete time system.

I guess a "natural", initial choice is always the **Euler forward** approximation (page 17 at the paper above), where the (complex) continuous time variable s is appromimated with:

s ≈ (z - 1) / h

where z is the complex variable after a Z-transform of a discrete time signal and h our usual (and small) time step parameter.

The Euler forward approximation is just the simplest approximation, though the author above warns:

*Forward (explicit) Euler approach is numerically not efficient (very small integration intervals T required).*

*Complex algorithms designed for efficient numerical integration not applicable to real-time control systems."*

(T is our h, which is generally 0.01[ sec ] here)

<br/>

However, and the following examples will empirically show this, I wouldn't worry too much about this - at least for now.

It's a different story if sampling interval h in a real-world system cannot be small enough compared to the relevant time constants of a process. This can easily happen if an automation system is cramped with lots of tasks (often for (non-smart) cost reasons, specifically in the long term --> process quality!).
As a rule of thumb I would like to see a ratio of **h ≈ 1/10·(smallest relevant time constant of the process)** or better, but 1 : 8 is also still OK, something which I might explain at a later time.

And there's more to it why Euler forward may be acceptable as I've found out by my simulation experiments.

<br/>

But back to math:

PT2: G(s) = X(s)/U(s) = b0 / (s² + a1·s + a0)

(for simplicity, here I shorten variable x1 in the source code to just x (time-domain) and X (frequency domain), respectively).

When replacing s with (z - 1)/h, one gets:

X(z) / U(z) = b0 / ((z - 1)²/h² + a1·(z - 1)/h + a0)

...

<br/>

The "trick" here is to put the term of X with the highest power of z on the **left hand side of the equation**, and only this term, and all the rest to the other side:

starting with: X / U = b0 / ((z - 1)²/h³ + a1·(z - 1)/h + a0) ...

Tools for **symbolic math** (like SymPy, online: https://live.sympy.org/) can help with this delicate and error-prone task.

These kind of equations can easily become huge! It's easy to make mistakes while getting a form which can actually be coded:

...finally retrieves this:

X·z² = h²·b0·U + (2 - h·a1)·z·X + (a1·h - a0·h² - 1)·X

<br/>

Now there's another "trick" to know. We learned from the Karl Johan Åström-implementation of a PID controller that execution order is essential: https://github.com/PLC-Programmer/Python/tree/master/systems%2Cfilters_and_feedback-controls/PID-control#execution-order

One can now play to shift the z-operator to a lower or higher order if helpful with the later implementation of the whole simulation loop. Remember:

* forward shift operation: z·x(k) ≡ x(k+1)
* backward shift operation: z^(-1)·x(k) ≡ x(k-1)

For a PI controller (to replicate the controller of chapter 3.5 as mentioned above), I just leave away the D-term of the "Karl Johan Åström"-PID (KJA) controller:

```
for k in range(1,STEPS):
    # P-term:
    P_KJA = Kp_KJA * (w_1 - x1[k])
    # D-term with derivative of measurement only:
    ...
    # controller output:
    u_KJA[k] = P_KJA + I_KJA[k]  # + D_KJA[k]
    ...
    # I-term:
    if k < STEPS-1:
        ...
        control_error = w_1 - x1[k]

        I_KJA[k+1] = I_KJA[k] + Kp_KJA*h/Tn*control_error

    # process:
    if k < STEPS-1:
        # Euler forward emulation of
        # process transfer function G(s) = b0 / (s**2 + a1*s + a0)
        # ...
        x1[k+1] = h**2.0 * b0 * u_KJA[k-1]\
                  + (2.0 - h*a1) * x1[k]\
                  + (a1*h - a0 * h**2.0 - 1.0) * x1[k-1]
        ...
```

See the complete program form here: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e_PI-control-loop_with_2nd_order_lag_process_non-state-space.py

Simulation outcome looks like this (with the later given task to find controller pameters Kp and Tn that will lead to a first overshoot of 20% at second 2.5 for a unit step change of the set point, that is reference for the controller): 

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e_PI-control-loop_with_2nd_order_lag_process_non-state-space.png)

This looks very promising.

<br/>

### 2/ How good is the Karl Johan Åström-PI controller with the Euler forward emulation of the PT2 process?

Here's the state-space algorithm as presented in the "Computer simulation of controls"-book linked above in my Python implementation: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space.py

```
def DIFF_EQU(x_1, x_2, x_3):
    ...
    u_   = Kp * (w_1 - x_1 + x_3/Tn)  # controller output
    ...
    f1  = x_2
    f2  = b0 * u_ - a0 * x_1 - a1 * x_2  # PT2 term
    f3  = w_1 - x_1
    return f1, f2, f3

# system simulation loop:
for k in range(1,STEPS):
    # using Runge–Kutta method:
    x1_ = x1ss[k-1]
    x2_ = x2ss[k-1]
    x3_ = x3ss[k-1]
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k1 = h * f1_
    l1 = h * f2_
    m1 = h * f3_
    x1_ = x1ss[k-1] + k1 / 2.0
    x2_ = x2ss[k-1] + l1 / 2.0
    x3_ = x3ss[k-1] + m1 / 2.0
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k2 = h * f1_
    l2 = h * f2_
    m2 = h * f3_
    x1_ = x1ss[k-1] + k2 / 2.0
    x2_ = x2ss[k-1] + l2 / 2.0
    x3_ = x3ss[k-1] + m2 / 2.0
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k3 = h * f1_
    l3 = h * f2_
    m3 = h * f3_
    x1_ = x1ss[k-1] + k3 / 2.0
    x2_ = x2ss[k-1] + l3 / 2.0
    x3_ = x3ss[k-1] + m3 / 2.0
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k4 = h * f1_
    l4 = h * f2_
    m4 = h * f3_
    x1_ = x1ss[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
    x2_ = x2ss[k-1] + (l1 + 2.0*l2 + 2.0*l3 + l4) / 6.0
    x3_ = x3ss[k-1] + (m1 + 2.0*m2 + 2.0*m3 + m4) / 6.0

    x1ss[k] = x1_
    x2ss[k] = x2_
    x3ss[k] = x3_
```

The generated outcome of this algorithm based on the Runge-Kutta 4th order method ("RK4") is the **benchmark** for my non-state-space implementations for the given control task.

State variable x1ss[k] is equivalent to measurement x1[k] and so the later should follow x1ss[k] as close as possible.

By the way and as mentioned here : https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods:

_..the simplest Runge–Kutta method is the (forward) Euler method.._

<br/>

When I overlay the curves of both simulations, the result looks like this:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space%20-%20a.png)

That's really close in my opinion, specifically when you consider that in real life measurement x1 can by really noisy.

Here's the difference plot of x1ss and x1: 

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space%20-%20b.png)

<br/>

### 3/ Next exercise: PI controller in Euler forward emulation

Let's put the usual, independent differential equation of a PI controller:

Gc(s) = U(s)/E(s) = Kp·(1 + 1/(Tn·s))

..into an Euler forward approximated form, same like before with the PT2 process, so again with: s ≈ (z - 1) / h:

U(z)/E(z) = Kp·(1 + 1/(Tn·(z - 1) / h))

Following the instructions given above, the difference equation for the PI controller becomes:

u(k) = Kp·e(k) + Kp·(h/Tn - 1)·e(k-1) + u(k-1)

..which looks like a very efficient _control policy_: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e2_PI-control-loop_with_2nd_order_lag_process_Euler_forward.py

When I compare its generated step response with the step response of the Åström-PI-controller, both curves look the same: 

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e2_PI-control-loop_with_2nd_order_lag_process_Euler_forward.png)

And indeed, the difference of both curves to the state-space benchmark is the same for 6 digits:

_absolute, maximum relative difference of Euler forward-PI controller - state-space solution: 0.865217%_

_absolute, maximum relative difference of Karl Johan Åström-PI controller - state-space solution: 0.865217%_

A closer look of the difference between the two curves only shows very low numbers, here from time step 0 to 12:

_0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00,  2.16840434e-19,  4.33680869e-19,  8.67361738e-19, 1.73472348e-18,  3.46944695e-18,  5.20417043e-18,  6.93889390e-18,..._

So, we can assume: the Karl Johan Åström-PI controller is some form of a Euler forward-PI controller. (I can't say anything on the Åström-**PID** controller; the related paper approximates the D-term by a _backward_ difference, compared to the approximation of the I-term by _forward_ difference.)

### 4/ Exercise: everything in Euler backward emulation

Let's put both parts of the control loop into an Euler backward approximated form.

Then the approximation of both differential equations, for controller and process, will be done with: s ≈ (z - 1) / (z·h)

After some tedious re-formulations (with the help of SymPy; see from above), the PI controller difference equation becomes this:

u(k) = Kp·(1 + h/Tn)·e(k) - Kp·e(k-1) + u(k-1)

..and the difference equation of the PT2 process this monster:

x(k) = 1/(1 + a1·h + a0·h²) · (b0·h²·u(k) + (2 + a1·h)·x(k-1) - x(k-2))

The astute observer of the related source code (https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process/3.5e3_PI-control-loop_with_2nd_order_lag_process_Euler_backward.py) may have noticed that I shifted the z-operator up by one time step on both sides of the equation of the process:

```
x1_bw[k+1] = (b0*h**2.0*u_bw[k] + (2.0+a1*h)*x1_bw[k] - x1_bw[k-1])\
                     / (1.0 + a1*h + a0*h**2.0)
```

The resulting maximum difference of x1 to the state-space benchmark solution came indeed down to:

_absolute, maximum relative difference of Euler backward-PI controller - state-space solution: 0.608077%_

..albeit only a little bit from 0.865217% with the Euler forward solution for controller and process.

So, the statement above ("Forward (explicit) Euler approach is numerically not efficient...") holds some truth in this purely simulation based setup with a sufficiently small time step parameter h.

<br/>

Extra piece of fun: what happens if we mix a PI controller in Euler backward emulation with a process in Euler forward emulation?

Like this for example:

```
for k in range(1,STEPS):
    control_error[k] = w_1 - x1_bw[k]
    # Euler backward-PI controller:
    u_bw_ef[k] = Kp*(1 + h/Tn)*control_error[k] - Kp*control_error[k-1] + u_bw_ef[k-1]
    # process:
    if k < STEPS-1:
        # Euler forward emulation of
        # process transfer function G(s) = b0 / (s**2 + a1*s + a0)
        # ...
        x1_bw_ef[k+1] = h**2.0 * b0 * u_bw_ef[k-1]\
                  + (2.0 - h*a1) * x1_bw_ef[k]\
                  + (a1*h - a0 * h**2.0 - 1.0) * x1_bw_ef[k-1]
```

Well, it gets a little bit worse:

_absolute, maximum relative difference of (Euler backward-PI controller+ Euler forward process) - state-space solution: 0.870828%_

So for now I suggest to not mix both emulations if not needed.

<TBD -- bilinear/Tustin transformation>





