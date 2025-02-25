2025-02-25: story continues here at "PI control of an (undamped) 2nd order lag process": https://github.com/PLC-Programmer/Python/tree/master/systems%2Cfilters_and_feedback-controls/PID-control/PI%20control%20of%20an%20(undamped)%202nd%20order%20lag%20process

------

2025-02-20

(a)

From now on, I will only use the "Karl Johan Åström"-PID algorithm.

\
(b)

There's no good production-ready PID controller without:

1. possibility to bound its output u(t) (here to simulate the physical limitations in the real world)

2. implementation of an anti-windup mechanism for its integrator

Here's a new version for this: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/3.5d_real_PID-control_with_PT1_process_u-bounded_anti-windup.py

\
To 1.:

The simple process model in use (1st order lag, "PT1") doesn't explicitly model a real, physical actuator, but in reality there's most probably one. So, an actuator - with its physical limits - is implicitly modeled within the process model.

Here are some more variables shown of an actually unbounded controller since it's minimum and maximum output values in this use case are only 0.0 and 1.10 respectively, values which are safely within the output bounds:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5d_real_PID-control_with_PT1_process_u-bounded_anti-windup.py%20-%20U_MAX_1.50.png)

Now a test with narrower bounds of -1.0 <= u(t) <- +1.0:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5d_real_PID-control_with_PT1_process_u-bounded_anti-windup.py%20-%20U_MAX_1.00.png)

\
To 2.:

The unbounded PID algorithm makes another problem visible: **integrator saturation**!

While the controller output is still moving, the actuator might have already hit a limit for some time or even a long time!

=> long story short: the closed control loop may become unstable or even more unstable and under most, if not even all circumstances the control quality will suffer!

Luckily **anti-windup** is taking care of this unwanted phenomenon: as soon as the controller (or actuator respectively) hits a limit any further integration at the I-term of the PID controller is stopped:

```
    # controller output:
    u_KJA[k] = P_KJA + I_KJA[k] + D_KJA[k]

    ANTIWINDUP = False
    if BOUND is True:
        if abs(u_KJA[k]) > U_MAX:
            u_KJA[k] = np.sign(u_KJA[k]) * U_MAX
            ANTIWINDUP = True
```
...
```
        if ANTIWINDUPACTION is True:
            if ANTIWINDUP is False:
                # control error which goes into the integrator of controller
                control_error = w_1 - x1[k]
            else:
                control_error = 0.0  # no more control error in this case
        else:
            control_error = w_1 - x1[k]
```


\
Positive effect: the value of the integrator output stays well below the maximum value of the case without anti-windup and controller output u(t) can leave its upper bound much sooner and the target process value of 1.0 is reached with less overshooting. See the blue, dotted lines in the two diagrams shown above.

Here's a diagram which compares the measurment values of both cases: (practically) unbounded controller output in orange color versus bounded controller output in blue color:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5d1_real_PID-control_with_PT1_process_compare_bounded_with_unbounded.png)

While the anti-windup effect is most profound at the initial set point step change, it's still (accidently, albeit slightly) visible at the measurement disturbance from second 15 on.

------

2025-02-19b

re "..do you want to **increase** the value of parameter Tv, that is Td, significantly for "good" derivative action.."

--> this is the reason why at the bottom of the program I provisionally provided these print statements:
```
print("min(D_KJA)=",min(D_KJA))
print("max(D_KJA)=",max(D_KJA))
```
..to see what are the maximum (peak) contributions of the D-term during a test. It may give a clue where to move the value of parameter Td (= Tv). This is especially true when the parameter value is so off (with a certain PID-implementation) that the test *numerically* derails.

\
re  "This derivative term can easily cause numerous problems with a practical implementation, and be it only with the numerical stability of an algorithm. Textbook implementations, including their online versions, may sooner or later show their weaknesses.":

Here's an example: this otherwise great and concise book on control engineering (in German language only: https://link.springer.com/book/10.1007/978-3-658-45897-3), features (at least in the prior 16th edition available to me) this PID algorithm on page 339:
```
c1 = Kp*h/(h + Tf)*(1 + (h + Tf)/(2.0*Tn) + (Tf + Tv)/h)
c2 = Kp*h/(h + Tf)*(h/(2.0*Tn) + 2.0*(Tf + Tn)/h - 1.0)
c3 = Kp*h/(h + Tf)*((Tf + Tv)/h - Tf/(2.0*Tn))
d1 = 1.0 + Tf/(h + Tf)
d2 = Tf/(h + Tf)
...
    e_zr[k] = w_1 - x1_zr[k]
    y[k] = d1*y[k-1] - d2*y[k-2] + c1*e_zr[k] + c2*e_zr[k-1] + c3*e_zr[k-2]
...
```

Parameters c1, c2, c3, d1 and d2 look plausible with 10.53, 11.38, 10.47, 1.95 and 0.95 respectively.

However, only after step number 25 or so error term e_zr[k] starts to go wild with the same control parameter values as used with the "van de Logt"- and "Karl Johan Åström"-algos:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5c1_real_PID-control_with_PT1_process%20--%20Zacher%2C%20Reuter%2C%20page%20339%2C%2016.%20Auflage.png)

(for this quick diagram I used these online tools: https://www.graphreader.com/plotter + https://pinetools.com/generate-list-numbers)

.. which makes the Python interpreter complaining like this after a while: "..RuntimeWarning: overflow encountered in scalar multiply"

So, it makes perfect sense to consider **only changes of the measurement for the control** error at some terms of the textbook PID algorithm, specifically for the D-term, at least in some situations.

However and on the other hand, with a simulation where a perfect "measurement" without noise has to start from point 0.0 a set point change might be the only trigger to start the simulation!

   

------

2025-02-19a

Without further ado here's the Python source code of a PID controller according to the two algorithms mentioned below: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/3.5c_real_PID-control_with_PT1_process.py

This is only a first (workable) version without bells and whistles. So you won't probably want to use it in a real production environment (textbooks almost never care about this aspect).

Here are the responses of both algos to a unit step jump of the set point (= reference):

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5c_real_PID-control_with_PT1_process%20a.png)

It's easy to see that both algorithms work differently, even with the same set of parameter values.

The Karl Johan Åström looks more responsive. I have an idea for the reason.

The "van de Logt"-algorithm in orange color (there's also a hompage which I didn't check out in detail: http://www.vandelogt.nl/uk_regelen_pid.php) calculates the D-term like this (in my interpretation):

```
    # D-term with derivative of measurement x1 only:
    # otherwise (eq.11c) from source below
    if k < STEPS-1:
        lpf[k] = (2.0*Tf - h)/(2.0*Tf + h)*lpf[k-1]\
                 - h / (h + 2.0*Tf) * (x1[k] + x1[k-1])

    # (eq. 12) from:
    # PID Controller Calculus, V3.20
    # ir. drs. E.H.W. van de Logt
    if k < STEPS-2:
        u[k] = u[k-1] + Kp*((e[k] - e[k-1])\
                            + h*e[k]/Tn\
                            + Tv/h*(lpf[k] - 2.0*lpf[k-1] + lpf[k-2]))
```

Time-discrete variable *lpf* for "low-pass filter" is of interest here. It features a second order difference equation (from eq. 12: Tv/h·(lpf[k] - 2.0·lpf[k-1] + lpf[k-2])).

The paper in the link below explains where this comes from. Van de Logt takes the differential equation of a PID controller (the "textbook" or "independent" PID equation) and differentiates it, which shows at (eq. 09). This yields this D-term in continuous time:

Td·d²(e(t))/dt

(with me in "German": time constant Tv = Td)

The idea to differentiate this form of a PID controller is common, see here for example at equation (2): http://bestune.50megs.com/typeABC.htm

Then he transforms the resulting differential equation into the time-discrete domain, where the D-term ends up as:

Td/h·(e[k] - 2·e[k-1] + e[k-2]) from eq. 10; h = my symbol for the sampling time period, here 0.01[sec]

He then applies a simple first-order low-pass filter to this D-term.

\
Compare this to the D-term of the "Karl Johan Åström"-algorithm:

```
    # D-term with derivative of measurement only:
    D_KJA[k] = Tv_KJA/(Tv_KJA + Tf)\
               * (D_KJA[k-1] - Kp_KJA*N*(y_KJA[k] - y_KJA[k-1]))
```

It only goes back one time step to calculate a new value for the D-term! 

(y[k] is equivalent to x1[k])

Mr Åström apparently also pulls out another "trick" out of his hat: the I-term (integration term) is only calculated **after** the new value of the controller output u[k]! I changed that to see if I break things. I broke things.

Beside this, it's clear that the "Karl Johan Åström"-implementation of a PID controller is a rougher approximation of the continuous, real, textbook-PID controller. It's explained in his paper for example for the D-term from equation (6.16.) onwards.

What I've learned so far is that with **discretizations** of principially analog systems **details** matter a lot.

\
So, I took the "van de Logt"-algorithm and manually tuned it so that it approximately matches the step response curve of the "Karl Johan Åström"-algorithm:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5c_real_PID-control_with_PT1_process%20b.png)

Not really the same parameters values!

\
Next test was to add some noise to the measurement to see if the step responses are still OK:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5c_real_PID-control_with_PT1_process%20c.png)

They are.

\
Then from second 15 on I applied a pulse disturbance to see if this non-tuned set of parameter values can cope with it:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5c_real_PID-control_with_PT1_process%20d.png)

Some observations:

* the "van de Logt"-algorithm keeps both swings more in control, but shows again its specific sluggishness

* the "Karl Johan Åström"-algorithm (with the original set of parameter values at the curve in red color) allows for more overshooting, but brings the measurement back to the set point value faster

* the blue curve shows that these parameter values are not optimally tuned to match the original curve in red color

\
At last, I made this test: what happens if I leave away the D-term from the "Karl Johan Åström"-PID controller, but keep the same value for Kp and Tn; see the black curve:

![plot](https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/pictures/3.5c_real_PID-control_with_PT1_process%20e.png)

While its step resonse looks OK, this controller allows for more overshooting at the pulse disturbance. So, D-terms have an effect!

I say this because I saw it numerous times that people activated the D-term of their controller only to be disappointed. The D-term is apparently the tricky one: do you want to **increase** the value of parameter Tv, that is Td, significantly for "good" derivative action only to wait for an unexpected disturbance at the measurement which may result in a controller to go out of control and to play havoc with your plant?!?

\
By the way: the amplitude of this pulse disturbance is only 0.01 (at *DIST_VAL*), but it causes suprisingly hefty reactions.

I have no clue why this is so.
 

------

2025-02-18

Implementation of a PID controller as a **state-space controller** isn't so easy (for me) but apparently doable: 

(0)

2022: **State-Space PID: A Missing Link Between Classical and Modern Control**

https://ieeexplore.ieee.org/document/9933758

by Wen Tan, Wenjie Han, Jiaohu Xu

\
I successfully implemented a PI controller as a state-space controller, but failed so far with an "ideal" PID controller (no low-pass filtering on the derivative term, D-term), not to mention a "real" PID controller (low-pass filtering on the D-term).

Source (0) clearly shows the way to success for a PID controller as a state-space controller: the derivative of the plant output, that is the derivative of measurement y(t), has to be estimated by a suitable observer, that is an **extended state observer** with proper observer gain. This is true for the ideal PID controller as well as the real PID controller.

The "trick" here is that in such a way **no plant model** is being used. Because if a plant model is used to (implicitly) estimate state vector **x(t)** of a PID controller, the controller is no longer independent of the plant model. And the later may be wrong sooner or later!

\
The problem for a practical implementation of a (digital) PID controller comes from here: transfer function of the (ideal) D-term = **Kd·s**

This derivative term can easily cause numerous problems with a practical implementation, and be it only with the numerical stability of an algorithm. Textbook implementations, including their online versions, may sooner or later show their weaknesses.

Of course, the problem with the D-term has been solved for ages. One only has to know where to look at.

I've found two source, a fact that shows how widely ideas can differ to tackle the problem of a derivative term of the control error!

(a) PID Controller Calculus, V3.20, ir. drs. E.H.W. van de Logt, 2011:

http://www.vandelogt.nl/datasheets/pid_controller_calculus_v320.pdf

This PDF document even features C source code for its PID algorithms.

\
and

(b) a very classical source by Karl Johan Åström from 2002 which may have been published in other forms in later works:

https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom.html/astrom-ch6.pdf

https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf

\
I may already say that the algorithm in (b) according to the unnamed(!) equations on page 242 under "Summarizing we find that the PID controller can be approximated by" is great, maybe even unbeatable in its simplicity!

I will show this later in some diagrams.

Both algorithms can be easily implemented with a D-term only of the measurement, like quite ubiquitous, or with a D-term on the *control error = set point - measurement* like in many textbooks.

And the calculated step responses of both algorithms show another problem which may be notoriously underrated until this day: 

- you cannot simply take one set of (tuned) PID controller parameters (Kp, Ti, Td, Td_lpf) from one implementation and naively expect them to also work in a different setup but with the same label "PID"!
