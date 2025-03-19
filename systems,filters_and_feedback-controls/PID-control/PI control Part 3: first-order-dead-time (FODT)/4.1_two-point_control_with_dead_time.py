# -*- coding: utf-8 -*-
"""
2025-01-26

4.1_two-point_control_with_dead_time.py


tests: OK!


idea: Computer-Simulation von Regelungen - book, 1999, E.-G. Feindt: chapter 4.1
"""

import numpy as np
import matplotlib.pyplot as plt


# ini values:
tstart = 0.0
h = 0.01  # delta t

# process parameters:
V  = 1.0
T1 = 1.0


STEPS = 5000
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)
x1 = np.zeros([STEPS])
x2 = np.zeros([STEPS])

# for deadtime:
STEPS_DT = 100  # deadtime = 1 second = 100 * 0.01
# input signal:
u = np.zeros([STEPS_DT], dtype = float)

w_1   = 0.0
x1[0] = 0.0


def DIFF_EQU(x_2, y_):
    '''
    calculate f1,f2: first order ODE's
    '''
    f1  = x_2
    f2  = (V * y_ - x_2) / T1  # y = output of dead time term
    return f1, f2


# system simulation loop:
for k in range(1,STEPS):

    # controller algo:
    e1  = w_1 - x1[k-1]
    if e1 > 0:
        u_ = 1.0
    else:
        u_ = -1.0

    # deadtime simulation loop:
    u[0] = u_
    for m in range(STEPS_DT-1,0,-1):
        u[m] = u[m-1]
    y = u[-1]  # output of deadtime process block


    # using Runge–Kutta method:
    x1_ = x1[k-1]
    x2_ = x2[k-1]
    f1_, f2_ = DIFF_EQU(x2_, y)

    k1 = h * f1_
    l1 = h * f2_
    x1_ = x1[k-1] + k1 / 2.0
    x2_ = x2[k-1] + l1 / 2.0
    f1_, f2_ = DIFF_EQU(x2_, y)

    k2 = h * f1_
    l2 = h * f2_
    x1_ = x1[k-1] + k2 / 2.0
    x2_ = x2[k-1] + l2 / 2.0
    f1_, f2_ = DIFF_EQU(x2_, y)

    k3 = h * f1_
    l3 = h * f2_
    x1_ = x1[k-1] + k3 / 2.0
    x2_ = x2[k-1] + l3 / 2.0
    f1_, f2_ = DIFF_EQU(x2_, y)

    k4 = h * f1_
    l4 = h * f2_
    x1_ = x1[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
    x2_ = x2[k-1] + (l1 + 2.0*l2 + 2.0*l3 + l4) / 6.0

    x1[k] = x1_
    x2[k] = x2_


plt.plot(t,x1)
plt.title('On-off control: dead time + IT1 process:\nstable limit cycle of a non-linear system')
plt.xlabel('t')
plt.ylabel('system state x1')
plt.legend([f'ini values: x1[0]={x1[0]:.1f}, x2[0]={x2[0]:.1f}'])
plt.grid()
plt.show()

# xy-plot: x1, x2
plt.plot(x1,x2)
plt.title('On-off control: dead time + IT1 process:\nstable limit cycle of a non-linear system')
plt.xlabel('system state x1')
plt.ylabel('system state x2')
plt.legend([f'ini values: x1[0]={x1[0]:.1f}, x2[0]={x2[0]:.1f}'])
plt.grid()
plt.show()


# end of 4.1_two-point_control_with_dead_time.py
