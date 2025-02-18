# -*- coding: utf-8 -*-
"""
2025-02-14/15/17/18

3.5c_PID-control_with_first-order-dead-time_(FODT)_process.py


rev.2025-02-18b: taking proper care of state variable x2
rev.2025-02-18a: option for bounded controller output; anti-windup option
rev.2025-02-17: apply noise to x1(t) only at the end of the Runge–Kutta loop!


Based on program:
  3.5_PI-control-loop_with_2nd_order_lag_process.py
  2.3_dead_time-simulation.py


tests: OK!


idea: Computer-Simulation von Regelungen - book, 1999, E.-G. Feindt:
      chapter "8.3 Regelkreis-Simulation mit Approximation der Sprungfunktion"
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager


# ini values:
tstart = 0.0
h = 0.01  # delta t

BOUND = True  # True = bounded controller output (for dead time process)
U_MAX = 2.0   # absolute

ANTIWINDUPACTION = True

# PT1 process coefficients:
b0 = 1.0
a0 = 1.0  # T1 = 1/a0
# numerator polynomial:   b0
# denominator polynomial: s + a0
# => X(s)/U(s) = b0/(s+a0)
# => x' + a0*x = b0*u

DELAY = 50   # dead time in time steps: 50 = 0.5[sec] <=> 1[sec}

# ideal PID controller parameters:
Kp = 1.5     # proportional gain
Tn = a0/1.2  # time constant of I-term
Tv = 0.005   # time constant of D-term: org 0.005
#
# idea how to implement an ideal PID controller:
#   u(t) = Kp*(e(t) + 1/Tn*∫e(t)*dt + Tv*e(t)') <-- controller output
#   e(t) = w1 - x1 <-- control error
#   => e' = w1' - x1' =  w1' - x2
#   x5 := ∫(w1 - x1)*dt
#   w1' = 1/h <-- see from chapter 8.3, h = program stepping
#   => u = Kp*(w1 - x1 + 1/Tn*x5 + Tv*(w1' - x2))
#   x2 := x1' = f1
#   x5' = w1 - x1 = f5  (see from line above)


STEPS = 500  # org 500
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)
x1 = np.zeros([STEPS])
x2 = np.zeros([STEPS])
x1no = np.zeros([STEPS])  # no dead time
x2no = np.zeros([STEPS])  # no dead time
x1nc = np.zeros([STEPS])  # no control

x5 = np.zeros([STEPS])    # for I term of PID controller
x5no = np.zeros([STEPS])  # same without dead time


w_1 = 1.0  # jump of reference value at t = 0

SIGMA = 0.0035  # sigma of noisy measurement
x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)


def DIFF_EQU(x_1, x_2, x_5):
    '''
    calculate f1,f2: first order ODE's
    same for both calculations: with or without process dead time
    '''
    # for testing process dead time: fixed controller output
    # u = 1.0

    w1_deriv = 1.0/h

    # PID controller output:
    u  = Kp*(w_1 - x_1 + 1/Tn*x_5 + Tv*(w1_deriv - x_2))

    ANTIWINDUP = False
    if BOUND is True:
        if abs(u) > U_MAX:
            u = np.sign(u) * U_MAX
            ANTIWINDUP = True

    f1  = b0 * u - a0 * x_1  # PT1 process

    f2 = f1  # x2 := x1' = f1

    if ANTIWINDUPACTION is True:
        if ANTIWINDUP is False:
            f5 = w_1 - x_1  # control error which goes into the integrator of controller
        else:
            f5 = 0.0  # no more control error in this case
    else:
        f5 = w_1 - x_1

    return f1, f2, f5, u


# for monitoring:
u_dt = np.zeros([STEPS])

# system simulation loop with process dead time:
for k in range(1,STEPS):
    # implement the process dead time:
    for dt in range(DELAY-1,0,-1):
        x1[dt] = x1[dt-1]
        x2[dt] = x2[dt-1]
    # x1 with DELAY = 5 and no control for example:
    # array([0., 0., 0., 0., 0.,
    # 0.00995846, 0.01981775, 0.02957885, 0.03924275, 0.04881041,...
    # this implementation is done like
    # in chapter "2.4 Simulation einer Totzeit"

    # using Runge–Kutta method:
    x1_ = x1[k-1]
    x2_ = x2[k-1]
    x5_ = x5[k-1]
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k1 = h * f1_
    l1 = h * f2_
    o1 = h * f5_
    x1_ = x1[k-1] + k1 / 2.0
    x2_ = x2[k-1] + l1 / 2.0
    x5_ = x5[k-1] + o1 / 2.0
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k2 = h * f1_
    l2 = h * f2_
    o2 = h * f5_
    x1_ = x1[k-1] + k2 / 2.0
    x2_ = x2[k-1] + l2 / 2.0
    x5_ = x5[k-1] + o2 / 2.0
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k3 = h * f1_
    l3 = h * f2_
    o3 = h * f5_
    x1_ = x1[k-1] + k3 / 2.0
    x2_ = x2[k-1] + l3 / 2.0
    x5_ = x5[k-1] + o3 / 2.0
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k4 = h * f1_
    l4 = h * f2_
    o4 = h * f5_
    x1_ = x1[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
    x2_ = x2[k-1] + (l1 + 2.0*l2 + 2.0*l3 + l4) / 6.0
    x5_ = x5[k-1] + (o1 + 2.0*o2 + 2.0*o3 + o4) / 6.0

    x1[k] = x1_ * x1_noise[k-1]
    x2[k] = x2_
    x5[k] = x5_

    u_dt[k] = u_



# system simulation loop without process dead time:
for k in range(1,STEPS):
    # using Runge–Kutta method:
    x1_ = x1no[k-1]
    x2_ = x2no[k-1]
    x5_ = x5no[k-1]
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k1 = h * f1_
    l1 = h * f2_
    o1 = h * f5_
    x1_ = x1no[k-1] + k1 / 2.0
    x1_ = x1no[k-1] + k1 / 2.0
    x5_ = x5no[k-1] + o1 / 2.0
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k2 = h * f1_
    l2 = h * f2_
    o2 = h * f5_
    x1_ = x1no[k-1] + k2 / 2.0
    x1_ = x1no[k-1] + k1 / 2.0
    x5_ = x5no[k-1] + o2 / 2.0
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k3 = h * f1_
    l3 = h * f2_
    o3 = h * f5_
    x1_ = x1no[k-1] + k3 / 2.0
    x1_ = x1no[k-1] + k1 / 2.0
    x5_ = x5no[k-1] + o3 / 2.0
    f1_, f2_, f5_, u_ = DIFF_EQU(x1_, x2_, x5_)

    k4 = h * f1_
    l4 = h * f2_
    o4 = h * f5_
    x1_ = x1no[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
    x2_ = x2no[k-1] + (l1 + 2.0*l2 + 2.0*l3 + l4) / 6.0
    x5_ = x5no[k-1] + (o1 + 2.0*o2 + 2.0*o3 + o4) / 6.0

    x1no[k] = x1_ * x1_noise[k-1]
    x2no[k] = x2_
    x5no[k] = x5_



def DIFF_EQU2(x_1):
    '''
    calculate f1: first order ODE
    '''
    # for testing process dead time: fixed controller output
    u = 1.0

    f1  = b0 * u - a0 * x_1  # PT1 process
    return f1

# system simulation loop without control, only the FODT response:
for k in range(1,STEPS):
    # implement the process dead time:
    for dt in range(DELAY-1,0,-1):
        x1nc[dt] = x1nc[dt-1]

    # using Runge–Kutta method:
    x1_ = x1nc[k-1]
    f1_ = DIFF_EQU2(x1_)

    k1 = h * f1_
    x1_ = x1nc[k-1] + k1 / 2.0
    f1_ = DIFF_EQU2(x1_)

    k2 = h * f1_
    x1_ = x1nc[k-1] + k2 / 2.0
    f1_ = DIFF_EQU2(x1_)

    k3 = h * f1_
    x1_ = x1nc[k-1] + k3 / 2.0
    f1_ = DIFF_EQU2(x1_)

    k4 = h * f1_
    x1_ = x1nc[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0

    x1nc[k] = x1_ * x1_noise[k-1]



deadtime = DELAY*h

plt.suptitle(f'(ideal) PID controller with a FODT process\
\nFODT: 1st order lag w/ T1={a0:.1f}[sec], dead time of {deadtime}[sec]', y = 1.04)
plt.title(f'with Gaussian noise with sigma={SIGMA}', y = 1.0)
# plt.title(f'PT1: T1={a0:.1f}[sec] without measurement noise', y = 1.0)

plt.plot(t,x1, label=f'control parameters: Kp={Kp:.2f}, Tn={Tn:.3f}, Tv={Tv:.4f}\
\nreference jump to 1.0 at time 0', color="orange")

if BOUND is True:
    plt.plot(t,u_dt, label='bounded controller output with dead time process',\
    color="orange", linestyle='--')
else:
    plt.plot(t,u_dt, label='unbounded controller output with dead time process',\
    color="orange", linestyle='--')

plt.plot(t,x5, label='state variable for integral part I of controller',\
color="orange", linestyle=':')
plt.plot([],[], label=f'anti-windup in action: {ANTIWINDUPACTION}',\
color="red")

# plt.plot(t,x1no, label='same PID control without process dead time\
# \n(only noisy 1st order lag)')

# plt.plot(t,x1nc, label='only noisy FODT response without control', color="grey")

plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')

font = font_manager.FontProperties(size=9)
plt.legend(loc="upper right", prop=font)

plt.grid()
plt.show()


# end of 3.5c_PID-control_with_first-order-dead-time_(FODT)_process.py
