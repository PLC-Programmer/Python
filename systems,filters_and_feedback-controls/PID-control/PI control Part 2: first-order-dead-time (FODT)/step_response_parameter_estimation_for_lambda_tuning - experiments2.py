# -*- coding: utf-8 -*-
"""
2025-03-10

step_response_parameter_estimation_for_lambda_tuning - experiments2.py


target: bringing back noise into the original process simulation loop of x1


Based on program:
    step_response_parameter_estimation_for_lambda_tuning - experiments.py


tests: OK!


"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager


# ini values:
tstart = 0.0
h = 0.01  # delta t


# 3 x PT1 process coefficients:
b0 = 1.0
a0 = 1.0
a1 = 1.0
# numerator polynomial:   b0
# denominator polynomial: (a1*s + a0)*(a1*s + a0)*(a1*s + a0)


STEPS = 1250  # 1250 for process step responses
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)


x1       = np.zeros([STEPS])  # measurement for controller
x1_clean = np.zeros([STEPS])  # clean measurement = clean process output as the benchmark

u  = np.zeros([STEPS])  # process input signal
u[:] = 1.0
u[0] = 0.0  # here, process input jump at second point in time


NOISE = True
SIGMA = 0.01  # org 0.01; sigma of noisy measurement

if NOISE is True:
    x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)  # multiplicative noise
else:
    x1_noise = np.zeros([STEPS])
    x1_noise[:] = 1.0


# long constant expressions for the process:
A0 = a0**3*h**3 - 3*a0**2*a1*h**2 + 3*a0*a1**2*h
A1 = a0**2*h**2 - 2*a0*a1*h + a1**2


# process simulation loop:
for k in range(2,STEPS):

    # process:
    if k < STEPS-1:
        # Euler forward emulation of
        # process transfer function G(s) = b0 / ((a0 + a1*s)*(a0 + a1*s)*(a0 + a1*s))
        # ...
        # again using SymPy; see at:
        #   3.5f2_PI-control-loop_with_2xPT1_Euler_forward_process.py

        x1[k+1] = 1.0/a1**3.0 * ((a1**3.0 - A0)*x1[k-2]/x1_noise[k-2] \
                                     - 3.0*a1**2.0*(a0*h - a1)*x1[k]/x1_noise[k] \
                                     - 3.0*a1*A1*x1[k-1]/x1_noise[k-1] \
                                     + b0*h**3.0*u[k-2])

        x1_clean[k+1] = 1.0/a1**3.0 * ((a1**3.0 - A0)*x1_clean[k-2] \
                                     - 3.0*a1**2.0*(a0*h - a1)*x1_clean[k] \
                                     - 3.0*a1*A1*x1_clean[k-1] \
                                     + b0*h**3.0*u[k-2])

        if NOISE is True:
            x1[k+1] = x1[k+1] * x1_noise[k+1]



plt.suptitle('Process parameter estimation of PT1 process in series (3x)', y = 1.0)
plt.title('3xPT1 process: b0=1.0, a0=1.0, a1=1.0 [sec] (Euler forward)', y = 1.0)

plt.plot(t,x1, label='3xPT1: step response to reference jump to 1.0 at time 0')
plt.plot(t,x1_clean, label='step response without noise')

if NOISE is True:
    plt.plot([],[], label = f'sigma of noisy x1 (multiplicative): {SIGMA}')

plt.xlabel(f't [sec], stepping h={h} [sec]')
plt.ylabel('system state x1 (process output, measurement)')

font = font_manager.FontProperties(size=8)
## plt.legend(loc="upper right", prop=font)
plt.legend(loc="lower right", prop=font)
# plt.legend(prop=font)

plt.grid()
plt.show()


# end of step_response_parameter_estimation_for_lambda_tuning - experiments2.py
