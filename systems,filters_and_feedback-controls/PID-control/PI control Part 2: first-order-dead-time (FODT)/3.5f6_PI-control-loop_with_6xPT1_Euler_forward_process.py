# -*- coding: utf-8 -*-
"""
2025-03-17

3.5f6_PI-control-loop_with_6xPT1_Euler_forward_process.py


Based on program:
    3.5f3_PI-control-loop_with_3xPT1_Euler_forward_process.py


tests: Okish; but a transient disturbance apparently looks not realistic



ideas:

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

# 6 x PT1 process coefficients:
b0 = 1.0
a0 = 1.0
a1 = 1.0
# numerator polynomial:   b0
# denominator polynomial: (a1*s + a0)^6


# PI controller parameters:
#   using Lambda Tuning here only for 1xPT1:
#     Controller Gain Kc = tau/(gp x (taucl + td))
#     Integral Time Ti = tau
# =>
tau=3.57  # [sec]; from computer evaluation of the process step response
gp=1.0
taucl_factor = 3.0  # org: 3.0; 1.5 for faster control
taucl=taucl_factor*tau
td=2.83  # [sec]; from computer evaluation of the process step response
# =>
Kp = tau/(gp * (taucl + td))
Tn = tau


# for real PID control:
Tf = 0.2  # time constant of lowpass filter of 1st order (PT1) [sec]


STEPS = 5000  # 1250 for process step responses
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)

lpf = np.zeros([STEPS])  # low-pass filter of first order for D-term

w_1 = 1.0  # jump of reference value at t = 0

# transient disturbance:
TRANSIENT_DISTURBANCE = False
t_dist_start_t = 15.0
t_dist_stop_t  = 17.0
t_dist_start   = t_dist_start_t / h
t_dist_stop    = t_dist_stop_t / h
DIST_VAL = 0.000000001


NOISE = True
FILTERING = False
SIGMA = 0.01  # org 0.01; sigma of noisy measurement
if NOISE is True:
    x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)  # multiplicative noise
else:
    x1_noise = np.zeros([STEPS])
    x1_noise[:] = 1.0


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
# this implementation only has derivative action on the measurement:
# read from page 222:
# "The parameter c is normally zero to avoid large transients in the control
#  signal due to sudden changes in the setpoint."
# => set point weight factor c := 0, also in the active code below:

x1       = np.zeros([STEPS])  # measurement for controller
x1_clean = np.zeros([STEPS])  # clean measurement = clean process output as the benchmark

I_KJA = np.zeros([STEPS])  # I-term of Karl Johan Åström-PID controller
D_KJA = np.zeros([STEPS])  # D-term of Karl Johan Åström-PID controller
# r_KJA = np.zeros([STEPS])  # set point
# r_KJA[:] = 1.0
# r_KJA[0] = 0.0  # here, set point jump at second point in time
u_KJA = np.zeros([STEPS])  # controller output for monitoring

Kp_KJA = Kp*1.0
# Tv_KJA = Tv*1.0
# Tf  = Tv/N =>
Tf_KJA = Tf*1.0
# N = Tv_KJA/Tf_KJA


# long constant expressions for the process:
A0 = a0**6*h**6 - 6*a0**5*a1*h**5 + 15*a0**4*a1**2*h**4 - 20*a0**3*a1**3*h**3 \
     + 15*a0**2*a1**4*h**2 - 6*a0*a1**5*h + a1**6
A1 = 6*a1*(a0**5*h**5 - 5*a0**4*a1*h**4 + 10*a0**3*a1**2*h**3 \
     - 10*a0**2*a1**3*h**2 + 5*a0*a1**4*h - a1**5)
A2 = 15*a1**2*(a0**4*h**4 - 4*a0**3*a1*h**3 + 6*a0**2*a1**2*h**2 \
     - 4*a0*a1**3*h + a1**4)
A3 = 20*a1**3*(a0**3*h**3 - 3*a0**2*a1*h**2 + 3*a0*a1**2*h - a1**3)
A4 = 15*a1**4*(a0**2*h**2 - 2*a0*a1*h + a1**2)
A5 = 6*a1**5*(a0*h - a1)
A6 = a1**6


for k in range(5,STEPS):
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
        # process transfer function G(s) = b0 / ((a0 + a1*s)^6)
        # ...
        # again using SymPy; see at:
        #   3.5f2_PI-control-loop_with_2xPT1_Euler_forward_process.py

        x1[k+1] = 1/A6 * (b0*h**6*u_KJA[k-5]
                          - A5*x1[k]/x1_noise[k]
                          - A4*x1[k-1]/x1_noise[k-1]
                          - A3*x1[k-2]/x1_noise[k-2]
                          - A2*x1[k-3]/x1_noise[k-3]
                          - A1*x1[k-4]/x1_noise[k-4]
                          - A0*x1[k-5]/x1_noise[k-5])

        x1_clean[k+1] = 1/A6 * (b0*h**6*u_KJA[k-5]
                                - A5*x1_clean[k]
                                - A4*x1_clean[k-1]
                                - A3*x1_clean[k-2]
                                - A2*x1_clean[k-3]
                                - A1*x1_clean[k-4]
                                - A0*x1_clean[k-5])

        if NOISE is True:
            x1[k+1] = x1[k+1] * x1_noise[k+1]

        # transient disturbance:
        if TRANSIENT_DISTURBANCE is True:
            if t_dist_start <= k <= t_dist_stop:
                x1[k+1] += DIST_VAL



plt.suptitle('Simulation of PI controller with a PT1 process in series', y = 1.0)
plt.title('6xPT1 process: b0=1.0, a0=1.0, a1=1.0 [sec] (Euler forward)', y = 1.0)

# plt.plot(t,x1, label=f'PID controller (KJ Åström algo):\
# \nKp={Kp_KJA:.2f},Tn={Tn:.2f},Tv={Tv_KJA:.2f},Tf={Tf_KJA:.2f}')

plt.plot(t,x1, label=f'PI controller (Karl Johan Åström algorithm):\
\nKp={Kp_KJA:.4f},Tn={Tn:.4f}, which are based on:\
\n  tau={tau} [sec], td={td} [sec]\
\n  (computer reading w/ noisy measurement)\
\n  taucl factor={taucl_factor} (my setting)\
\nreference jump to 1.0 at time 0\
\nanti-windup in action: {ANTIWINDUPACTION}')

"""
if BOUND is True:
    plt.plot(t,u_KJA, label=f'bounded controller output: -{U_MAX}..+{U_MAX}',\
    color="red", linestyle='--')
else:
    plt.plot(t,u_KJA, label='unbounded controller output',\
    color="red", linestyle='--')
"""

# plt.plot(t,I_KJA, label='variable for integral part of controller',\
# linestyle=':', color="#1f77b4")  # matplotlib default plot color

if NOISE is True:
    plt.plot([],[], label = f'sigma of noisy x1 (multiplicative): {SIGMA}')

if TRANSIENT_DISTURBANCE is True:
    plt.plot([],[], label=f'transient disturbance of {DIST_VAL}\
\nadded on x1 from {t_dist_start_t} to {t_dist_stop_t} sec')

plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')


font = font_manager.FontProperties(size=8.5)
# plt.legend(loc="upper right", prop=font)
plt.legend(prop=font)

plt.grid()
plt.show()


# print("\nmin(D_KJA)=",min(D_KJA))
# print("max(D_KJA)=",max(D_KJA))
print("\nmin(u_KJA)=",min(u_KJA))
print("max(u_KJA)=",max(u_KJA))


# end of 3.5f6_PI-control-loop_with_6xPT1_Euler_forward_process.py
