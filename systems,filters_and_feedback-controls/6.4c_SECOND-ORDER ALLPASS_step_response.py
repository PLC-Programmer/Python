# -*- coding: utf-8 -*-
"""
2025-02-09

6.4c_SECOND-ORDER ALLPASS_step_response.py


------------------------------------------------------------------------------
In this program this transfer function of a second-order allpass filter

  H(s) = Y(s)/U(s) = (s^2 - w0/Q*s + w0^2) / (s^2 + w0/Q*s + w0^2)

is to be implemented and simulated for its step response.

Initial, arbitrary choices:
  - w0 = 1.0: center frequency (pole resonant frequency) of the filter [rad/s]
  - Q  = 1.0: quality factor of the filter

Introducing a dummy variable Z(s), or z(t) in time domain, a "trick" taken
from working with (abstract) state-space representations:

Y(s) = ((s^2 - w0/Q*s + w0^2) * 1/(s^2 + w0/Q*s + w0^2))*U(s)

Z(s) := 1/(s^2 + w0/Q*s + w0^2)*U(s)
Y(s)  = (s^2 - w0/Q*s + w0^2)*Z(s)

=> in time domain:
z'' + w0/Q*z' + w0^2*z = u
x1 := z
x2 := z'
=>
x1' = x2
=>
x2' + w0/Q*x2 + w0^2*x1 = u
=> canonical state-space form with state-space matrices A + B:
[x1'    [0          1    [x1     [0
 x2'] =  -w0^2  -w0/Q] *  x2] +  1]*u

y = z'' - w0/Q*z' + w0^2*z
=>
y = x2' - w0/Q*x2 + w0^2*x1
  = -w0/Q*x2 - w0^2*x1 + u - w0/Q*x2 + w0^2*x1
  = -2*w0/Q*x2 + u
=> state-space matrices C + D:
y = [0  -2*w0/Q]*[x1 x2]T + [1]*u  # T = transpose

------------------------------------------------------------------------------


"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sig


# ini values:
tstart = 0.0
h = 0.01  # delta t, that is 10[ms]

# filter parameters:
Q = 1.0   # [1]
w0 = 1.0  # [rad/sec]

# system  + input matrix x' = A*x + B*u of the filter:
A = np.array([[0.0, 1.0 ],
              [-w0**2.0,-w0/Q]])

B = np.array([[0,1.0]]).T  # B must have same number of rows!

# output  + feed-forward matrix y = C*x + D*u of the filter:
C = np.array([[0.0, -2.0*w0/Q]])

D = np.array([[1.0]]).T

sys_ap2 = sig.StateSpace(A, B, C, D)


STEPS = 1500
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)
x1 = np.zeros([STEPS])
x2 = np.zeros([STEPS])
y_sim  = np.zeros([STEPS])


# ini state:
x0 = [0.0, 0.0]
# unit step response of the system:
t, y_sig = sig.step(sys_ap2,x0,t)



def DIFF_EQU(x_1, x_2):
    '''
    calculate f1,f2: first order ODE's
    '''
    # unit step as process input:
    u = 1.0

    f1 = x_2
    f2 = -w0**2.0*x_1 - w0/Q*x_2 + u
    y__ = -2.0*w0/Q*x_2 + u
    return f1, f2, y__


# system simulation loop:
for k in range(1,STEPS):
    # using Runge–Kutta method:
    x1_ = x1[k-1]
    x2_ = x2[k-1]
    f1_, f2_, y_ = DIFF_EQU(x1_, x2_)

    k1 = h * f1_
    l1 = h * f2_
    x1_ = x1[k-1] + k1 / 2.0
    x2_ = x2[k-1] + l1 / 2.0
    f1_, f2_, y_ = DIFF_EQU(x1_, x2_)

    k2 = h * f1_
    l2 = h * f2_
    x1_ = x1[k-1] + k2 / 2.0
    x2_ = x2[k-1] + l2 / 2.0
    f1_, f2_, y_ = DIFF_EQU(x1_, x2_)

    k3 = h * f1_
    l3 = h * f2_
    x1_ = x1[k-1] + k3 / 2.0
    x2_ = x2[k-1] + l3 / 2.0
    f1_, f2_, y_ = DIFF_EQU(x1_, x2_)

    k4 = h * f1_
    l4 = h * f2_
    x1_ = x1[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
    x2_ = x2[k-1] + (l1 + 2.0*l2 + 2.0*l3 + l4) / 6.0

    x1[k] = x1_
    x2[k] = x2_

    y_sim[k] = y_


plt.suptitle('Simulation of a second order allpass filter: unit step response', y = 1.0)
plt.title(f'parameters: center frequency ω0={w0}[rad/s], quality factor Q={Q}', y = 1.0)

plt.plot(t,y_sim, label=f'ODE simluation: y(t): system output\
\nstarting from initial state {x1[0]:.1f}, {x2[0]:.1f}')

plt.plot(t,y_sig, label=f'modeling with scipy.signal.StateSpace + .step\
\nstarting from initial state {x0[0]}, {x0[1]}', linestyle='--')

plt.xlim((h,tstop))  # do not plot from origin: matplotlib starts at 0.0,0.0!

plt.xlabel('t [s]')
plt.legend()
plt.grid()
plt.show()


# end of 6.4c_SECOND-ORDER ALLPASS_step_response.py
