2025-02-19: work in progress

------

2025-02-19

Without further ado here's the Python source code of a PID controller according to the two algorithms mentioned below: *3.5c_real_PID-control_with_PT1_process.py*: https://github.com/PLC-Programmer/Python/blob/master/systems%2Cfilters_and_feedback-controls/PID-control/3.5c_real_PID-control_with_PT1_process.py

This is only a first (workable) version without bells and whistles. So you won't probably want to use it in a real production environment (textbooks almost never care about this aspect).

Here the individual responses to a unit step jump of the set point (= reference) of both algos:

![plot](./pictures/3.5c_real_PID-control_with_PT1_process%20a.png]


 







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











 







