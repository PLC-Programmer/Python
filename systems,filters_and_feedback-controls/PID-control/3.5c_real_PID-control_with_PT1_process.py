# -*- coding: utf-8 -*-
"""
2025-02-18

3.5c_real_PID-control_with_PT1_process.py


tests: OK!


ideas:
    1/ PID Controller Calculus, V3.20, ir. drs. E.H.W. van de Logt, 2011
    2/ chapter 6 "PID Control" from "Control System Design" by Karl Johan Åström, 2002
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager


# ini values:
tstart = 0.0
h = 0.01  # delta t


# PT1 process coefficients:
b0 = 1.0
a0 = 1.0  # T1 = 1/a0
# numerator polynomial:   b0
# denominator polynomial: s + a0
# => X(s)/U(s) = b0/(s+a0)
# => x' + a0*x = b0*u
T1 = 1.0/a0


# ideal PID controller parameters:
Kp = 1.0     # proportional gain
Tn = a0/1.0  # time constant of I-term
Tv = 2.0     # time constant of D-term

# for real PID control:
Tf = 0.2  # time constant of lowpass filter of 1st order (PT1) [sec]


STEPS = 3000
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


# system simulation loop:
for k in range(2,STEPS):

    e[k] = w_1 - x1[k]

    # D-term with derivative of measurement x1 only:
    # otherwise (eq.11c) from source below
    if k < STEPS-1:
        lpf[k] = (2.0*Tf - h)/(2.0*Tf + h)*lpf[k-1]\
                 - h / (h + 2.0*Tf) * (x1[k] + x1[k-1])

    # (eq. 12) from:
    # PID Controller Calculus, V3.20
    # ir. drs. E.H.W. van de Logt
    if k < STEPS-2:
        u[k] = u[k-1] + Kp*((e[k] - e[k-1])\
                            + h*e[k]/Tn\
                            + Tv/h*(lpf[k] - 2.0*lpf[k-1] + lpf[k-2]))
    # PT1 process:
    if k < STEPS-1:
        x1[k+1] = (x1[k] + h/(T1+h) * (u[k] - x1[k])) # * x1_noise[k-1]
        
        # transient disturbance:
        if t_dist_start <= k <= t_dist_stop:
            x1[k+1] += DIST_VAL



# same algo, but with parameters to be tuned to match the
# the step response of the Karl Johan Åström-PID controller below
x1_  = np.zeros([STEPS])
u_   = np.zeros([STEPS])
e_   = np.zeros([STEPS])
lpf_ = np.zeros([STEPS])

Kp_ = 0.85
Tn_ = 0.85
Tv_ = 1.0
Tf_ = 0.2

# system simulation loop:
for k in range(2,STEPS):

    e_[k] = w_1 - x1_[k]

    # D-term with derivative of measurement x1 only:
    # otherwise (eq.11c) from source below
    if k < STEPS-1:
        lpf_[k] = (2.0*Tf_ - h)/(2.0*Tf_ + h)*lpf_[k-1]\
                 - h / (h + 2.0*Tf_) * (x1_[k] + x1_[k-1])

    # (eq. 12) from:
    # PID Controller Calculus, V3.20
    # ir. drs. E.H.W. van de Logt, 2011
    if k < STEPS-2:
        u_[k] = u_[k-1] + Kp_*((e_[k] - e_[k-1])\
                            + h*e_[k]/Tn_\
                            + Tv_/h*(lpf_[k] - 2.0*lpf_[k-1] + lpf_[k-2]))
    # PT1 process:
    if k < STEPS-1:
        x1_[k+1] = (x1_[k] + h/(T1+h) * (u_[k] - x1_[k]))  # * x1_noise[k-1]
        
        # transient disturbance:
        if t_dist_start <= k <= t_dist_stop:
            x1_[k+1] += DIST_VAL



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

y_KJA = np.zeros([STEPS])  # measurement
I_KJA = np.zeros([STEPS])  # I-term of Karl Johan Åström-PID controller
D_KJA = np.zeros([STEPS])  # D-term of Karl Johan Åström-PID controller
r_KJA = np.zeros([STEPS])  # set point
r_KJA[:] = 1.0
r_KJA[0] = 0.0  # here, set point jump at second point in time

Kp_KJA = Kp*1.0
Tv_KJA = Tv*1.0
# Tf  = Tv/N =>
Tf_KJA = Tf*1.0
N = Tv_KJA/Tf_KJA

# this implementation only has derivative action on the measurement:
# read from page 222:
# "The parameter c is normally zero to avoid large transients in the control
#  signal due to sudden changes in the setpoint."
# => set point weight factor c := 0, also in the active code below:

for k in range(1,STEPS):
    # P-term:
    P_KJA = Kp_KJA * (w_1 - y_KJA[k])
    # D-term with derivative of measurement only:
    D_KJA[k] = Tv_KJA/(Tv_KJA + Tf)\
               * (D_KJA[k-1] - Kp_KJA*N*(y_KJA[k] - y_KJA[k-1]))

    # D-term with set point action:
    # D_KJA[k] = Tv_KJA/(Tv_KJA + Tf_KJA)\
    #    * (D_KJA[k-1]\
    #       - Kp_KJA*N*(y_KJA[k] - y_KJA[k-1])\
    #       + Kp_KJA*N*(r_KJA[k] - r_KJA[k-1]))

    # controller output:
    u_KJA = P_KJA + I_KJA[k] + D_KJA[k]

    # I-term:
    if k < STEPS-1:
        I_KJA[k+1] = I_KJA[k] + Kp_KJA*h/Tn*(w_1 - y_KJA[k])

    # PT1 process:
    if k < STEPS-1:
        y_KJA[k+1] = (y_KJA[k] + h/(T1+h)\
                      * (u_KJA - y_KJA[k])) # * x1_noise[k-1]

        # transient disturbance:
        if t_dist_start <= k <= t_dist_stop:
            y_KJA[k+1] += DIST_VAL


# same as PI controller:
y_KJApi = np.zeros([STEPS])  # measurement
I_KJApi = np.zeros([STEPS])  # I-term of Karl Johan Åström-PID controller
D_KJApi = np.zeros([STEPS])  # D-term of Karl Johan Åström-PID controller

for k in range(1,STEPS):
    # P-term:
    P_KJApi = Kp_KJA * (w_1 - y_KJApi[k])

    # controller output:
    u_KJApi = P_KJApi + I_KJApi[k]

    # I-term:
    if k < STEPS-1:
        I_KJApi[k+1] = I_KJApi[k] + Kp_KJA*h/Tn*(w_1 - y_KJApi[k])

    # PT1 process:
    if k < STEPS-1:
        y_KJApi[k+1] = (y_KJApi[k] + h/(T1+h)\
                      * (u_KJApi - y_KJApi[k])) # * x1_noise[k-1]

        # transient disturbance:
        if t_dist_start <= k <= t_dist_stop:
            y_KJApi[k+1] += DIST_VAL



plt.suptitle(f'(real) PID controllers with a PT1 process\
\nprocess: 1st order lag with T1={a0:.2f}[sec]', y = 1.04)
plt.title('D terms only based on measurement values', y = 1.00)

plt.plot(t,x1, label=f'control parameters: Kp={Kp:.2f},\
Tn={Tn:.2f},Tv={Tv:.2f},Tf={Tf:.2f}\
\nreference jump to 1.0 at time 0\
\ntransient disturbance of {DIST_VAL} from {t_dist_start_t} to {t_dist_stop_t} [sec]'\
, color="orange")

plt.plot(t,y_KJA, label=f'Karl Johan Åström-PID controller (KJA):\
\nKp={Kp_KJA:.2f},Tn={Tn:.2f},Tv={Tv_KJA:.2f},Tf={Tf_KJA:.2f}',\
color="red", linestyle='--')

plt.plot(t,x1_, label=f'modified parameters to match the KJA-controller:\
\nKp={Kp_:.2f},\
Tn={Tn_:.2f},Tv={Tv_:.2f},Tf={Tf_:.2f}',\
color="blue", linestyle='dotted')

plt.plot(t,y_KJApi, label=f'Karl Johan Åström-PI controller:\
\nKp={Kp_KJA:.2f},Tn={Tn:.2f}',\
color="black", linestyle='--')


plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')

font = font_manager.FontProperties(size=9)
# plt.legend(loc="upper right", prop=font)
plt.legend(prop=font)

plt.grid()
plt.show()


print("min(D_KJA)=",min(D_KJA))
print("max(D_KJA)=",max(D_KJA))

# end of 3.5c_real_PID-control_with_PT1_process.py
