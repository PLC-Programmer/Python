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

![plot](./systems%2Cfilters_and_feedback-controls/PID-control/3.5c_PID-control_with_first-order-dead-time_(FODT)_process%20--%20PID%2BPT1%20with%20noise%2C%20dead%20time%2C%20FODT%20response.png)

However, to not make things too complicated in one batch this example only features the ideal PID controller, that is without a filter term for its D-term.

\
(c)

As to be expected, the D-term can indeed accelerate the forcing of an otherwise sluggish process to a desired reference value.

On the other hand, dead time in such a process "makes satisfactory control more difficult to achieve" (see this excellent presentation (++): https://engineering.nyu.edu/mechatronics/Control_Lab/Criag/Craig_RPI/2002/Week2/First-Order_Process_Time_Delay_2002.pdf):

* with relevant dead time in the process the response of the closed control loop leads to more **overshooting** (all other things equal), a phenomenon - together with a noisy measurement in real life - which "always reduces the stability of a system and limits the achievable response time of the system" (++)



------

2025-02-11:

Added *6.4c_SECOND-ORDER ALLPASS_step_response.py* file https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/all-pass_filters/6.4c_SECOND-ORDER%20ALLPASS_step_response.py which shows how to successfully transform a transfer function G(s) with **derivatives in its numerator polynomial** into a state-space representation, for example for a second order all-pass filter:

![plot](./systems%2Cfilters_and_feedback-controls/all-pass_filters/pictures/G_s_transfer_function_AP2.png)

This state-space representation, composed of only first order ODE's (ordinary differential equations), is then simulated, that is integrated, with the Runge–Kutta method.

To check the result of the Runge–Kutta method, two functions from the SciPy signal processing toolbox (https://docs.scipy.org/doc/scipy/reference/signal.html) have been used: both calculations get the almost identical result for the unit step response! (except for the very first data point which the plot doesn't show for this reason)
\
\
When you have a transfer function G(s) with derivatives in its numerator polynomial, getting a correct state-space representation might not be so easy (at first). The problem is that in this case you have derivatives of the input signal u(t)!
\
\
For example:
 
G(s) = Y(s)/U(s) = (bn·sn + ... + b0) / (an·sn + ... + a0)

U(s) = system input (system == filter here)

Y(s) = system output

bn·sn + ... + b0 == numerator polynomial

an·sn + ... + a0 == denominator polynomial
 
=>

(an·sn + ... + a0)·Y(s) = (bn·sn + ... + b0)·U(s)  (A)

\
<ins>Solution to this problem:</ins>

As explained in the source code file, the solution is to introduce a **dummy variable Z(s)**, or z(t) in time domain.

To better accomplish this, we re-write equation (A) a little bit:

Y(s) = ((bn·sn + ... + b0) · 1/(an·sn + ... + a0))·U(s)

Now we introduce dummy variable Z(s)..

Z(s) := 1/(an·sn + ... + a0)·U(s)

..which will yield this equation for the output signal:

Y(s)  = (bn·sn + ... + b0)·Z(s)

Only now we can conveniently start to work in the time domain. For this example it means:

z'' + w0/Q·z' + w0^2·z = u <-- no more derivatives of input signal u (=u(t)) !!

(z' := dz/dt etc.)

Now we can introduce the usual state variables x1(t) and x2(t) (for the second order system):

x1 := z

x2 := z'

=>

x1' = x2

=>

x2' + w0/Q·x2 + w0^2·x1 = u

Output signal y(t) becomes:

y = z'' - w0/Q·z' + w0^2·z

=>

y = x2' - w0/Q·x2 + w0^2·x1

= -w0/Q·x2 - w0^2·x1 + u - w0/Q·x2 + w0^2·x1

= -2·w0/Q·x2 + u

\
If somebody is really familiar with the topic of state-space representation, this person might find the phase variable form or companion form of a state-space representation **directly** from the transfer function (I'm not. I have to go through all the ODE's first.):

![plot](./systems%2Cfilters_and_feedback-controls/all-pass_filters/pictures/fig5.png)

from: http://web.mit.edu/2.14/www/Handouts/StateSpace.pdf

(2.14 Analysis and Design of Feedback Control Systems, State-Space Representation of LTI Systems, Derek Rowell, October 2002)


