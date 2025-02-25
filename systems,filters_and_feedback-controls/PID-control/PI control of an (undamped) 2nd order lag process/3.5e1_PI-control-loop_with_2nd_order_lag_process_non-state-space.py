# -*- coding: utf-8 -*-
"""
2025-02-23/25

3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space.py


Based on programs:
    3.5e_PI-control-loop_with_2nd_order_lag_process_non-state-space.py
    3.5_PI-control-loop_with_2nd_order_lag_process.py


tests: OK!

  absolute, maximum relative difference of Åström-PI controller - state-space solution: 0.865217%


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
x1_KJA = np.zeros([STEPS])

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

x1_KJA = np.zeros([STEPS])  # measurement

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
    P_KJA = Kp_KJA * (w_1 - x1_KJA[k])
    # D-term with derivative of measurement only:
    # D_KJA[k] = Tv_KJA/(Tv_KJA + Tf)\
    #            * (D_KJA[k-1] - Kp_KJA*N*(x1_KJA[k] - x1_KJA[k-1]))

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
                control_error = w_1 - x1_KJA[k]
            else:
                control_error = 0.0  # no more control error in this case
        else:
            control_error = w_1 - x1_KJA[k]

        I_KJA[k+1] = I_KJA[k] + Kp_KJA*h/Tn*control_error

    # process:
    if k < STEPS-1:
        # Euler forward emulation of
        # process transfer function G(s) = b0 / (s**2 + a1*s + a0)
        # ...
        x1_KJA[k+1] = h**2.0 * b0 * u_KJA[k-1]\
                  + (2.0 - h*a1) * x1_KJA[k]\
                  + (a1*h - a0 * h**2.0 - 1.0) * x1_KJA[k-1]
        # x1_KJA[k+1] = x1_KJA[k+1] * x1_KJA_noise[k-1]

        # transient disturbance:
        # if t_dist_start <= k <= t_dist_stop:
            # x1_KJA[k+1] += DIST_VAL



# compare with the original state space solution from book's ch.3.5:
#
x1ss = np.zeros([STEPS])
x2ss = np.zeros([STEPS])
x3ss = np.zeros([STEPS])

def DIFF_EQU(x_1, x_2, x_3):
    '''
    calculate f1,f2,f3: first order ODE's
    '''
    # disturbance:
    # w_1 = 0.0

    u_   = Kp * (w_1 - x_1 + x_3/Tn)  # controller output

    # disturbance:
    # if k > tk:
        # t = k * h
        # u = u + np.sin(t)
        # u = u + 0.1
        # u = u + 1.0

    f1  = x_2
    f2  = b0 * u_ - a0 * x_1 - a1 * x_2  # PT2 term
    f3  = w_1 - x_1
    return f1, f2, f3

# system simulation loop:
for k in range(1,STEPS):
    # using Runge–Kutta method:
    x1_ = x1ss[k-1]
    x2_ = x2ss[k-1]
    x3_ = x3ss[k-1]
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k1 = h * f1_
    l1 = h * f2_
    m1 = h * f3_
    x1_ = x1ss[k-1] + k1 / 2.0
    x2_ = x2ss[k-1] + l1 / 2.0
    x3_ = x3ss[k-1] + m1 / 2.0
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k2 = h * f1_
    l2 = h * f2_
    m2 = h * f3_
    x1_ = x1ss[k-1] + k2 / 2.0
    x2_ = x2ss[k-1] + l2 / 2.0
    x3_ = x3ss[k-1] + m2 / 2.0
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k3 = h * f1_
    l3 = h * f2_
    m3 = h * f3_
    x1_ = x1ss[k-1] + k3 / 2.0
    x2_ = x2ss[k-1] + l3 / 2.0
    x3_ = x3ss[k-1] + m3 / 2.0
    f1_, f2_, f3_ = DIFF_EQU(x1_, x2_, x3_)

    k4 = h * f1_
    l4 = h * f2_
    m4 = h * f3_
    x1_ = x1ss[k-1] + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
    x2_ = x2ss[k-1] + (l1 + 2.0*l2 + 2.0*l3 + l4) / 6.0
    x3_ = x3ss[k-1] + (m1 + 2.0*m2 + 2.0*m3 + m4) / 6.0

    x1ss[k] = x1_
    x2ss[k] = x2_
    x3ss[k] = x3_
# end of compare with the original state space solution from book ch.3.5


# KJ Åström-PI controller - original state-space solution:
max_rel_diff_x1_KJA = max(abs(x1_KJA - x1ss))


plt.suptitle('Simulation of PI controller with a PT2 process (oscillating)', y = 1.04)
plt.title(f'non-state-space solution of process: ω0 = {omega0} [rad/s],\
\nD (damping) = {D}', y = 1.0)

# plt.plot(t,x1_KJA, label=f'PID controller (KJ Åström algo):\
# \nKp={Kp_KJA:.2f},Tn={Tn:.2f},Tv={Tv_KJA:.2f},Tf={Tf_KJA:.2f}')

plt.plot(t,x1_KJA, label=f'PI controller (Karl Johan Åström algorithm):\
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

plt.plot(t,x1ss,label="original state-space PI controller with same parameters (ch.3.5)",\
color = "orange")

# plt.plot(t,x1_KJA - x1ss, label=f'difference of state-space solution (ch.3.5)\
# \nand non-state-space solution with Åström-PI controller\
# \n+ Euler forward emulation of PT2 process\
# \nabsolute, maximum relative difference: {max_rel_diff_x1_KJA:.2%}',\
# color = "brown")

plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')

# plt.axvline(2.5, color='r', label =\
# f'20% overshoot at t=2.5 on an initial reference jump to {w_1}', ls = '--')

font = font_manager.FontProperties(size=8.5)
# plt.legend(loc="upper right", prop=font)
plt.legend(prop=font)

plt.grid()
plt.show()


# print("\nmin(D_KJA)=",min(D_KJA))
# print("max(D_KJA)=",max(D_KJA))
print("\nmin(u_KJA)=",min(u_KJA))
print("max(u_KJA)=",max(u_KJA))

print(f'\nabsolute, maximum relative difference of \
Åström-PI controller - state-space solution: {max_rel_diff_x1_KJA:.6%}')


# end of 3.5e1_PI-control-loop_with_2nd_order_lag_process_non-state-space.py
