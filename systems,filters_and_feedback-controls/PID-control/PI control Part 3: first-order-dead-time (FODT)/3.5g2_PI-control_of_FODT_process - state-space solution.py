# -*- coding: utf-8 -*-
"""
2025-03-19/20

3.5g2_PI-control_of_FODT_process - state-space solution.py


Based on programs:
    3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space.py
    3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process.py
    4.1_two-point_control_with_dead_time.py


tests: looks OK and very similar to the non-state-space solution in program
       3.5g1_PI-control-loop_with_first-order-dead-time_(FODT)_process.py
       Anti-windup and controller output bounding have not been tested here!


idea: Computer-Simulation von Regelungen - book, 1999, E.-G. Feindt: chapter 4.1
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager


# ini values:
tstart = 0.0
h = 0.01  # delta t

BOUND = True  # True = bounded controller output (for dead time process)
U_MAX = 2.0   # absolute; org 2.0

ANTIWINDUPACTION = True

# PT1 process coefficients (part of FODT):
# numerator polynomial:   b0
# denominator polynomial: (a1*s + a0)
# 2025-03-19: with K = 1.0, a0 = 1.0 => a1 = tau = 3.57, see from below
b0 = 1.0
a0 = 1.0
a1 = 3.57


# PI controller parameters:
#   using Lambda Tuning here only for 1xPT1:
#     Controller Gain Kc = tau/(gp x (taucl + td))
#     Integral Time Ti = tau
# =>
tau=3.57  # [sec]; from computer evaluation of the process step response
gp=1.0
taucl_factor = 1.5  # org: 3.0; 1.5 for faster control
taucl=taucl_factor*tau
td=2.83  # [sec]; from computer evaluation of the process step response
# =>
Kp = tau/(gp * (taucl + td))
Tn = tau

PERC63 = 0.63  # 63% of total PV change

DELAY = int(td/h)   # dead time in time steps


STEPS = 10000
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)
x1ss = np.zeros([STEPS])  # ss = state-space
x1ss_noise = np.zeros([STEPS])  # same variable with potential noise
x2ss = np.zeros([STEPS])

# input signal into deadtime process block:
u = np.zeros([DELAY], dtype = float)
u_ss = np.zeros([STEPS])  # just for reporting the controller output

r = np.zeros([STEPS])  # set point
r[:] = 1.0
r[0] = 0.0  # here, set point jump at second point in time


# w_1 = 1.0  # jump of reference value at t = 0


# transient disturbance:
TRANSIENT_DISTURBANCE = True
t_dist_start_t = 50.0
t_dist_stop_t  = 60.0
t_dist_start   = t_dist_start_t / h
t_dist_stop    = t_dist_stop_t / h
DIST_VAL = 0.001


# set point drop:
SP_DROP = False
if SP_DROP is True:
    SP_DROP_T = 50  # sec
    SP_DROP_STEP = int(SP_DROP_T/h)

NOISE = False
SIGMA = 0.01  # org 0.01; sigma of noisy measurement
if NOISE is True:
    x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)  # multiplicative noise
else:
    x1_noise = np.zeros([STEPS])
    x1_noise[:] = 1.0


def DIFF_EQU(x_1, y_, k_):
    '''
    calculate f1, f2: first order ODE's
    '''

    f1 = -a0/a1*x_1 + b0/a1*y_  # PT1 term
    f2 = r[k_] - x_1  # control error for I-part of PI controller
    return f1, f2


# system simulation loop:
for k in range(1,STEPS):

    # u_ = 1.0      # for simulation of a bump test

    # set point drop to 0.5:
    if SP_DROP is True:
        if k >= SP_DROP_STEP:
            r[k] = 0.5

    # PI controller algo:
    e1 = r[k] - x1ss[k-1] + x2ss[k-1]/Tn
    u_ = Kp * e1

    ANTIWINDUP = False
    if BOUND is True:
        if abs(u_) > U_MAX:
            u_ = np.sign(u_) * U_MAX
            ANTIWINDUP = True

    if ANTIWINDUPACTION is True:
        if ANTIWINDUP is False:
            u_ = Kp*(r[k] - x1ss[k-1] + x2ss[k-1]/Tn)
        else:
            u_ = Kp*(r[k] - x1ss[k-1])
            x2ss[k-1] = 0.0  # ??? correct
    else:
        u_ = Kp*(r[k] - x1ss[k-1] + x2ss[k-1]/Tn)


    u_ss[k] = u_  # just for reporting the controller output

    # deadtime simulation loop:
    u[0] = u_
    for m in range(DELAY-1,0,-1):
        u[m] = u[m-1]
    y = u[-1]  # output of deadtime process block

    # "de-noise": noise has no memory, at least not in this solution:
    x1ss[k-1] = x1ss[k-1] / x1_noise[k-1]


    # using Runge–Kutta method:
    x1_ = x1ss[k-1]
    x2_ = x2ss[k-1]
    f1_, f2_ = DIFF_EQU(x1_, y, k)

    k1 = h * f1_
    l1 = h * f2_
    x1_ = x1ss[k-1] + k1 / 2.0
    x2_ = x2ss[k-1] + l1 / 2.0
    f1_, f2_ = DIFF_EQU(x1_, y, k)

    k2 = h * f1_
    l2 = h * f2_
    x1_ = x1ss[k-1] + k2 / 2.0
    x2_ = x2ss[k-1] + l2 / 2.0
    f1_, f2_ = DIFF_EQU(x1_, y, k)

    k3 = h * f1_
    l3 = h * f2_
    x1_ = x1ss[k-1] + k3 / 2.0
    x2_ = x2ss[k-1] + l3 / 2.0
    f1_, f2_ = DIFF_EQU(x1_, y, k)

    k4 = h * f1_
    l4 = h * f2_
    x1_ = x1ss[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
    x2_ = x2ss[k-1] + (l1 + 2.0*l2 + 2.0*l3 + l4) / 6.0

    x1ss[k] = x1_ * x1_noise[k]
    x2ss[k] = x2_

    # transient disturbance:
    if TRANSIENT_DISTURBANCE is True:
        if t_dist_start <= k <= t_dist_stop:
            x1ss[k] += DIST_VAL

    x1ss_noise[k] =  x1ss[k]


plt.suptitle('Simulation of PI controller with a FODT process', y = 1.0)
plt.title(f'FODT: 1st order lag: tau={tau} sec, dead time={td} sec', y = 1.0)

plt.plot(t,x1ss_noise, label=f'PI controller (state-space implementation):\
\nKp={Kp:.4f},Tn={Tn:.4f}, which are based on:\
\n  tau={tau} [sec], td={td} [sec]\
\n  (computer reading w/ noisefree measurement)\
\n  taucl factor={taucl_factor} (my setting)\
\nreference jump to 1.0 at time 0\
\nanti-windup in action: {ANTIWINDUPACTION}')

if BOUND is True:
    plt.plot(t,u_ss, label=f'bounded controller output: -{U_MAX}..+{U_MAX}',\
    color="red", linestyle='--')
else:
    plt.plot(t,u_ss, label='unbounded controller output',\
    color="red", linestyle='--')


## plt.axhline(PERC63, color='r', label='63% of process value (PV)', linestyle='--')
## plt.axvline(0.0, color='magenta', label='start of bump test', linestyle='-.')
## plt.axvline(td, color='magenta', label=f'dead time td≈{td:.2f} [sec]:\
## \n  computer reading', linestyle='--')
## plt.axvline(tau+td, color='magenta', label=f'time constant tau≈{tau:.2f} [sec]:\
## \n  computer reading', linestyle=':')


if NOISE is True:
    plt.plot([],[], label = f'sigma of noisy x1 (multiplicative): {SIGMA}')

if TRANSIENT_DISTURBANCE is True:
    plt.plot([],[], label=f'transient disturbance of {DIST_VAL}\
\nadded on x1 from {t_dist_start_t} to {t_dist_stop_t} sec')

if SP_DROP is True:
    plt.plot(t,r, label=f'set point drop to 0.5 a {SP_DROP_T} [sec]')


plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')

font = font_manager.FontProperties(size=8)
plt.legend(loc="lower right", prop=font)
# plt.legend(prop=font)

plt.grid()
plt.show()



# end of 3.5g2_PI-control_of_FODT_process - state-space solution.py
