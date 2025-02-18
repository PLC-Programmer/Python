------

2025-02-18b

There's a Catch-22 in the ideal PID algorithm so far since in function *DIFF_EQU* the calculation of a new controller output value:
```
u  = Kp*(w_1 - x_1 + 1/Tn*x_5 + Tv*(w1_deriv - x_2))
```
..depends on state variable x2. And x2 has been defined to be nothing else than derivative x1' (= f1), which is only updated in a later command:
```
f1  = b0 * u - a0 * x_1  # PT1 process
```
 
However and in this use case, all the action comes from *w1_deriv = 1.0/h* and the **x2** numpy array is still filled with zeros at the end of the program!

Basically, for the use case of a set point (reference) jump, above command could be shortened to just:
```
u  = Kp*(w_1 - x_1 + 1/Tn*x_5 + Tv*(w1_deriv))
```

However, this is not sufficient for a proper D-term in a use case where measurement x1 is being moved by a process disturbance. Also in this case the controller's D-term should kick in.

So, I've updated the calculation of x2 with: 
```
f2 = f1                 # x2 := x1' = f1
```
in function *DIFF_EQU* and the system simulation loop accordingly.


------

2025-02-18a

Improved version of *3.5c_PID-control_with_first-order-dead-time_(FODT)_process.py* because there's a slight unrealistic aspect of this (ideal) PID algorithm so far: the controller output is without any bounds which is not realistic in nature!

(a)

This simple process model (1st order lag, "PT1") doesn't explicitly model a real, physical actuator, but in reality there's most probably one.

So, an actuator - which its physical limits - is implicitly modeled within the PID algorithm and/or process model.

Here's the curve of the unbounded controller output u(t) with the dashed line in orange color:

![plot](./pictures/3.5c_PID-control_with_first-order-dead-time_(FODT)_process%20-%20unbounded%20u.png)

\
Therefore the controller output is bounded now (arbitrarily to 2.0 for demonstration purposes):

![plot](./pictures/3.5c_PID-control_with_first-order-dead-time_(FODT)_process%20-%20bounded%20u.png)

```
BOUND = True  # True = bounded controller output (for dead time process)
U_MAX = 2.0   # absolute
```

...

```
    if BOUND is True:
        if abs(u) > U_MAX:
            u = np.sign(u) * U_MAX
```
                       
\
(b)

However, the unbounded PID algorithm makes another problem visible: **integrator saturation**!

While the controller output is still moving, the actuator might have already hit a limit for some time or even a long time!

=> long story short: the closed control loop may become unstable or even more unstable and under most, if not even all circumstances the control quality will suffer!


The dotted curve shows state variable x5, that is the output of the PID's integrator:

![plot](./pictures/3.5c_PID-control_with_first-order-dead-time_(FODT)_process%20-%20bounded%20u%2C%20I%20part.png)




\
Luckily **Anti-windup** is taking care of this unwanted phenomenon: as soon as the controller (or actuator respectively) hits a limit any further integration at the I-term of the PID controller is stopped:

```
    if ANTIWINDUPACTION is True:
        if ANTIWINDUP is False:
            f5 = w_1 - x_1  # control error which goes into the integrator of controller
        else:
            f5 = 0.0  # no more control error in this case
    else:
        f5 = w_1 - x_1
```

Positive effect: the value of the integrator output stays well below the maximum value in the case without anti-windup and controller output u(t) can leave its upper bound much sooner and the target process value of 1.0 is reached with less overshooting:

![plot](./pictures/3.5c_PID-control_with_first-order-dead-time_(FODT)_process%20-%20bounded%20u%2C%20anti-windup.png)
 

------

2025-02-17

Revised version of *3.5c_PID-control_with_first-order-dead-time_(FODT)_process.py*: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/3.5c_PID-control_with_first-order-dead-time_(FODT)_process.py

This was needed to improve the handling of noise on the measurement, that is state variable x1. Before, noise was added in every call of function *DIFF_EQU*, something which is not very logical.

Now, a noise signal is generated once and before a system simulation loop and applied commonly to all system simulation loops to compare apples to apples:

```
SIGMA = 0.0035  # sigma of noisy measurement
x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)
```

It's only added to x1 at the end of a system simulation loop, for example:

...

```
    x1[k] = x1_ * x1_noise[k-1]
    x2[k] = x2_
    x5[k] = x5_
```

 
------

2025-02-15

Added *3.5c_PID-control_with_first-order-dead-time_(FODT)_process.py* file https://github.com/PLC-Programmer/Python/blob/master/systems,filters_and_feedback-controls/PID-control/3.5c_PID-control_with_first-order-dead-time_(FODT)_process.py which implements an algorithm for a PID controller, a PID controller which actually includes a **derivative term** (D-term).

The excellent book "Computersimulation von Regelungen: Modellbildung und Softwareentwicklung" by Ernst-Guenther Feindt from 1999 (https://books.google.de/books/about/Computersimulation_von_Regelungen.html?id=745dDwAAQBAJ&redir_esc=y) shows a couple of examples (back in those days implemented in QBasic: https://en.wikipedia.org/wiki/QBasic) for a PI controller, but not a PID controller.

Additionally, I added **dead time** to a first-order lag process ("PT1"), which is commonly known as **FODT**.

\
(a)

Similar to the *6.4c_SECOND-ORDER ALLPASS_step_response.py* example, the transfer function Gc(s) of a PID controller features a **derivative in its numerator polynomial** (s·Kd), which may be a challenge for the computational implementation:

Gc(s) = Kp + Ki/s + s·Kd = U(s)/E(s)

U(s): controller output

E(s): control error = reference value - measurement value

For the implementation I followed the idea from the book mentioned above for a PI controller as described in chapter "3.3 Simulation des Reglers mit Integralanteil".
The controller output signal of an ideal PID controller can be described like this in the time domain:

u(t) = Kp·(e(t) + 1/Tn·∫e(t)·dt + Tv·e(t)'  with e(t)' = de(t)/dt etc.

Controller error is like usual: e(t) = w1 (reference) - x1 (state variable 1 = measurement = process output) which leads to its derivative: e' = w1' - x1' =  w1' - x2

Arbitrarily we assign state variable x5 to the integral over the control error: x5 := ∫(w1 - x1)·dt

Then the only open issue is a step (or jump) at the reference signal w1(t). Chapter "8.3 Regelkreis-Simulation mit Approximation der Sprungfunktion" just takes the reciprocal value of the program stepping (here 0.01\[sec\]) for it: w1' = 1/h

This works well as long as h is small enough compared to the time constants of the control loop (which is the case here).

Finally, the controller output signal for an ideal PID controller becomes:

u = Kp·(w1 - x1 + 1/Tn·x5 + Tv·(w1' - x2))

And here the remaining state variables and their derivatives:

x2 := x1' = f1 = b0·u - a0·x1 (the first-order lag process here)

x5' = f5 = w1 - x1

\
(b)

I added slight noise to the measurement signal x1 to visually show the effect of the controller's D-term on the control quality.

It's really visible if you compare it with the uncontrolled but smoother process response in gray color:

![plot](./pictures/3.5c_PID-control_with_first-order-dead-time_(FODT)_process%20--%20PID%2BPT1%20with%20noise%2C%20dead%20time%2C%20FODT%20response.png)
 
However, to not make things too complicated in one batch this example only features the ideal PID controller, that is without a filter term for its D-term.

\
(c)

As to be expected, the D-term can indeed accelerate the forcing of an otherwise sluggish process to a desired reference value.

On the other hand, dead time in such a process "makes satisfactory control more difficult to achieve" (see this excellent presentation (++): https://engineering.nyu.edu/mechatronics/Control_Lab/Criag/Craig_RPI/2002/Week2/First-Order_Process_Time_Delay_2002.pdf):

* with relevant dead time in the process the response of the closed control loop leads to more **overshooting** (all other things equal), a phenomenon - together with a noisy measurement in real life - which "always reduces the stability of a system and limits the achievable response time of the system" (++)

