2025-02-19: work in progress

------

2025-02-19

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

I say this because I saw it numerous times that people activated the D-term of their controller only to be disappointed. Yes, the D-term is apparently the tricky one: do you want to **increase** its parameter Tv, that is Td, significantly for "good" derivative action, only to wait for an unexpected disturbance at the measurement which may result in a controller to go out of control and to play havoc with your plant?!?

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
