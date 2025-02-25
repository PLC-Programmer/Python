# -*- coding: utf-8 -*-
"""
2025-02-23

3.5e_PI-control-loop_with_2nd_order_lag_process_non-state-space.py


Based on programs:
    3.5d_real_PID-control_with_PT1_process_u-bounded_anti-windup.py
    3.5_PI-control-loop_with_2nd_order_lag_process.py


tests: OK!!!


ideas:
    1/ Computer-Simulation von Regelungen - book, 1999, E.-G. Feindt: chapter 3.1...3.5
    2/ implement a non-state-space solution for a later non-state-space PID implementation

"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager


# ini values:
tstart = 0.0
h = 0.01  # delta t

BOUND = False  # True = bounded controller output (for dead time process)
U_MAX = 2.0   # absolute; org 2.0

ANTIWINDUPACTION = False

# PT2 process coefficients from: 3.5_PI-control-loop_with_2nd_order_lag_process.py
b0 = 1.3
a0 = 1.0
a1 = 1.6
# numerator polynomial:   b0
# denominator polynomial: s² + a1*s + a0
#  a1 = 2*D*ω0, a0 = ω0² (natural angular frequency, squared)
#  => ω0 = 1.0/sec, D = 0.8, which is underdamped
omega0 = np.sqrt(a0)
D = a1 / 2.0 / omega0
# PI controller parameters from book:
Kp = 1.2522
Tn = 1.3022


# for real PID control:
Tf = 0.2  # time constant of lowpass filter of 1st order (PT1) [sec]


STEPS = 1000
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)
x1 = np.zeros([STEPS])

u = np.zeros([STEPS])
e = np.zeros([STEPS])
lpf = np.zeros([STEPS])  # low-pass filter of first order for D-term

w_1 = 1.0  # jump of reference value at t = 0

# transient disturbance:
t_dist_start_t = 15.0
t_dist_stop_t  = 17.0
t_dist_start   = t_dist_start_t / h
t_dist_stop    = t_dist_stop_t / h
DIST_VAL = 0.01


SIGMA = 0.0035  # sigma of noisy measurement
x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)


###############################
#
# PID algorithm from:
# chapter 6 "PID Control" from "Control System Design" by Karl Johan Åström, 2002
# --> page 242 at "Summarizing we find that the PID controller can be approximated by"
#
# b and c are just constants: b for set point weighting
# r(t) = reference signal
# k = Kp
# y(t) = measurement = process variable to be controlled
# D-term: N for Td/N = Tf = time constant of PT1 filter, typically 8..20

x1 = np.zeros([STEPS])  # measurement

I_KJA = np.zeros([STEPS])  # I-term of Karl Johan Åström-PID controller
D_KJA = np.zeros([STEPS])  # D-term of Karl Johan Åström-PID controller
r_KJA = np.zeros([STEPS])  # set point
r_KJA[:] = 1.0
r_KJA[0] = 0.0  # here, set point jump at second point in time
u_KJA = np.zeros([STEPS])  # controller output for monitoring

Kp_KJA = Kp*1.0
# Tv_KJA = Tv*1.0
# Tf  = Tv/N =>
Tf_KJA = Tf*1.0
# N = Tv_KJA/Tf_KJA

# this implementation only has derivative action on the measurement:
# read from page 222:
# "The parameter c is normally zero to avoid large transients in the control
#  signal due to sudden changes in the setpoint."
# => set point weight factor c := 0, also in the active code below:

for k in range(1,STEPS):
    # P-term:
    P_KJA = Kp_KJA * (w_1 - x1[k])
    # D-term with derivative of measurement only:
    # D_KJA[k] = Tv_KJA/(Tv_KJA + Tf)\
    #            * (D_KJA[k-1] - Kp_KJA*N*(x1[k] - x1[k-1]))

    # controller output:
    u_KJA[k] = P_KJA + I_KJA[k]  # + D_KJA[k]

    ANTIWINDUP = False
    if BOUND is True:
        if abs(u_KJA[k]) > U_MAX:
            u_KJA[k] = np.sign(u_KJA[k]) * U_MAX
            ANTIWINDUP = True

    # I-term:
    if k < STEPS-1:
        if ANTIWINDUPACTION is True:
            if ANTIWINDUP is False:
                # control error which goes into the integrator of controller
                control_error = w_1 - x1[k]
            else:
                control_error = 0.0  # no more control error in this case
        else:
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
        # x1[k+1] = x1[k+1] * x1_noise[k+1]

        # transient disturbance:
        # if t_dist_start <= k <= t_dist_stop:
            # x1[k+1] += DIST_VAL



plt.suptitle('Simulation of PI controller with a PT2 process (oscillating)', y = 1.04)
plt.title(f'non-state-space solution of process: ω0 = {omega0} [rad/s],\
\nD (damping) = {D}', y = 1.0)

# plt.plot(t,x1, label=f'PID controller (KJ Åström algo):\
# \nKp={Kp_KJA:.2f},Tn={Tn:.2f},Tv={Tv_KJA:.2f},Tf={Tf_KJA:.2f}')

plt.plot(t,x1, label=f'PI controller (Karl Johan Åström algorithm):\
\nKp={Kp_KJA:.4f},Tn={Tn:.4f} (parameters from chapter 3.5)\
\nanti-windup in action: {ANTIWINDUPACTION}')

# if BOUND is True:
    # plt.plot(t,u_KJA, label=f'bounded controller output: -{U_MAX}..+{U_MAX}',\
    # color="red", linestyle='--')
# else:
    # plt.plot(t,u_KJA, label='unbounded controller output',\
    # color="red", linestyle='--')

# plt.plot(t,I_KJA, label='variable for integral part of controller',\
# linestyle=':', color="#1f77b4")  # matplotlib default plot color

plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')

plt.axvline(2.5, color='r', label =\
f'20% overshoot at t=2.5 on an initial reference jump to {w_1}', ls = '--')

font = font_manager.FontProperties(size=9)
# plt.legend(loc="upper right", prop=font)
plt.legend(prop=font)

plt.grid()
plt.show()


# print("\nmin(D_KJA)=",min(D_KJA))
# print("max(D_KJA)=",max(D_KJA))
print("\nmin(u_KJA)=",min(u_KJA))
print("max(u_KJA)=",max(u_KJA))


# end of 3.5e_PI-control-loop_with_2nd_order_lag_process_non-state-space.py
