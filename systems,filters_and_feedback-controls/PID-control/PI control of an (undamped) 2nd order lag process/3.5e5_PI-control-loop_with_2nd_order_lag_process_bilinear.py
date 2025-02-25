# -*- coding: utf-8 -*-
"""
2025-02-25

3.5e5_PI-control-loop_with_2nd_order_lag_process_bilinear.py


Based on programs:
    3.5e3_PI-control-loop_with_2nd_order_lag_process_Euler_backward.py


tests: OK, but worse than Euler forward or Euler backward control loop:

  absolute, maximum relative difference of bilinear PI controller - state-space solution: 0.996544%
  
  The original difference equation for the bilinear transform of the PT2 term
  leads to a significantly right-shifted step response and thus much worse result!
  (with an abs., max., rel. difference of 18.3% !!)
  So, I did a trick! See below.


ideas:
    1/ Computer-Simulation von Regelungen - book, 1999, E.-G. Feindt: chapter 3.1...3.5
    2/ Discrete Time Control Systems, Lino Guzzella, 2013

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


STEPS = 1000
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)


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
# PI algorithm of bilinear approximated controller:
x1_bilin = np.zeros([STEPS])    # measurement
control_error = np.zeros([STEPS])
u_bilin = np.zeros([STEPS])  # controller output

for k in range(1,STEPS):

    control_error[k] = w_1 - x1_bilin[k]

    # bilinear-PI controller:
    u_bilin[k] = 1/(2.0*Tn)*(Kp*(2.0*Tn + h)*control_error[k]\
                             + Kp*(h - 2.0*Tn)*control_error[k-1]\
                             + 2.0*Tn*u_bilin[k-1])

    # process:
    if k < STEPS-1:
        # bilinear approximation of
        # process transfer function G(s) = b0 / (s**2 + a1*s + a0)
        # ...
        x1_bilin[k+1] = 1/(a0*h**2.0 + 2*a1*h + 4.0) *\
                        (b0*h**2.0*(u_bilin[k] + 2.0*u_bilin[k-1] + u_bilin[k-2])\
                        - (2.0*a0*h**2.0 - 8.0) * x1_bilin[k]\
                        - (a0*h**2.0 - 2.0*a1*h + 4.0) * x1_bilin[k-1])
        # attention: x1_bilin[k+1]
        #   <==> u_bilin[k], which is shifted in sync with u_bilin[k+1]
        #        according to the original difference equation!!

        # x1_bilin[k+1] = x1_bilin[k+1] * x1_noise[k+1]

        # transient disturbance:
        # if t_dist_start <= k <= t_dist_stop:
            # x1_bilin[k+1] += DIST_VAL



# compare with the original state space solution from book's ch.3.5:
#
x1ss = np.zeros([STEPS])
x2ss = np.zeros([STEPS])
x3ss = np.zeros([STEPS])

def DIFF_EQU(x_1, x_2, x_3):
    '''
    calculate f1,f2,f3: first order ODE's
    '''
    u_   = Kp * (w_1 - x_1 + x_3/Tn)  # controller output

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


# bilinear PI controller - original state-space solution:
max_rel_diff_x1_bilin = max(abs(x1_bilin - x1ss))


plt.suptitle('Simulation of PI controller with a PT2 process (oscillating)', y = 1.04)
plt.title(f'non-state-space solution of process: ω0 = {omega0} [rad/s],\
\nD (damping) = {D} in bilinear approximation', y = 1.0)


plt.plot(t,x1_bilin, label=f'PI controller in bilinear approximation:\
\nKp={Kp:.4f},Tn={Tn:.4f} (parameters from chapter 3.5)\
\nabsolute, max. rel. difference to state-spaced solution: {max_rel_diff_x1_bilin:.2%}')


plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')

plt.axvline(2.5, color='r', label =\
f'20% overshoot at t=2.5 on an initial reference jump to {w_1}', ls = '--')

font = font_manager.FontProperties(size=9)
# plt.legend(loc="upper right", prop=font)
plt.legend(prop=font)

plt.grid()
plt.show()


print("\nmin(u_bilin)=",min(u_bilin))
print("max(u_bilin)=",max(u_bilin))

print(f'\nabsolute, maximum relative difference of \
bilinear PI controller - state-space solution: {max_rel_diff_x1_bilin:.6%}')

# end of 3.5e5_PI-control-loop_with_2nd_order_lag_process_bilinear.py
