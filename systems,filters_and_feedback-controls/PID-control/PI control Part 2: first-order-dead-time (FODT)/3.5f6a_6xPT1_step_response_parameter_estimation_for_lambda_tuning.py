# -*- coding: utf-8 -*-
"""
2025-03-12

3.5f6a_6xPT1_step_response_parameter_estimation_for_lambda_tuning.py


Based on program:
    step_response_parameter_estimation_for_lambda_tuning - experiments.py


tests: OK!!

NOISE_PRESENT = True
maximum gradient of potentially noisy measurement x1 = 0.00177
  at time = 5.03000
maximum gradient of noise-free x1 = 0.00176
  at time = 5.01000
SIGMA = 0.01
Savitzky-Golay filtering (2x):
  polynomial order = 1 (manually tuned)
  window size = 41 (manually tuned)


"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager

from scipy.signal import savgol_filter


# ini values:
tstart = 0.0
h = 0.01  # delta t


# 3 x PT1 process coefficients:
b0 = 1.0
a0 = 1.0
a1 = 1.0
# numerator polynomial:   b0
# denominator polynomial: (a1*s + a0)**6


# parameters for Lambda Tuning:
tau=2.4  # [sec]; just some ini value for now
td=0.8  # [sec]; just some ini value for now
gp=1.0   # everything stays within a unit jump
PERC63 = 0.63  # 63% of total PV change


STEPS = 1500  # 1500 for process step responses
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)


x1       = np.zeros([STEPS])  # measurement for controller
x1_clean = np.zeros([STEPS])  # clean measurement = clean process output as the benchmark

u  = np.zeros([STEPS])  # process input signal
u[:] = 1.0
u[0] = 0.0  # here, process input jump at second point in time


NOISE = True
FILTERING = True
SIGMA = 0.01  # org 0.01; sigma of noisy measurement

if NOISE is True:
    x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)  # multiplicative noise
else:
    x1_noise = np.zeros([STEPS])
    x1_noise[:] = 1.0


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


# process simulation loop:
for k in range(5,STEPS):

    # process:
    if k < STEPS-1:
        # Euler forward emulation of
        # process transfer function G(s) = b0 / ((a0 + a1*s)**6)
        # ...
        # again using SymPy; see at:
        #   3.5f2_PI-control-loop_with_2xPT1_Euler_forward_process.py

        x1[k+1] = 1/A6 * (b0*h**6*u[k-5]
                          - A5*x1[k]/x1_noise[k]
                          - A4*x1[k-1]/x1_noise[k-1]
                          - A3*x1[k-2]/x1_noise[k-2]
                          - A2*x1[k-3]/x1_noise[k-3]
                          - A1*x1[k-4]/x1_noise[k-4]
                          - A0*x1[k-5]/x1_noise[k-5])

        x1_clean[k+1] = 1/A6 * (b0*h**6*u[k-5]
                                - A5*x1_clean[k]
                                - A4*x1_clean[k-1]
                                - A3*x1_clean[k-2]
                                - A2*x1_clean[k-3]
                                - A1*x1_clean[k-4]
                                - A0*x1_clean[k-5])

        if NOISE is True:
            x1[k+1] = x1[k+1] * x1_noise[k+1]


x1_org = x1.copy()  # true copy


# calculating the gradient of the measurement:
x1_grad = np.gradient(x1_org)
x1_grad_no_noise = np.gradient(x1_clean)
x1_grad_org = x1_grad.copy()  # true copy


#############################################
#
# noise treatment:
#   1/ is measurment noisy?
#   2/ if yes, then filter it before calculating the gradient
#
# to 1/:
#   first idea: if there's no noise, then the gradient
#   must be positive throughout:
if min(x1_grad) < -0.00001:  # leave some room for error
    NOISE_PRESENT = True

    # Savitzky-Golay (SG) filtering of measurement:
    WINDOW_SIZE = 41  # 71: manual tuning --> OKish for one filter run
    POLYNOMIAL_ORDER = 1  # 1: manual tuning --> better tuning rule??

    # 1st filter run:
    x1_hat = savgol_filter(x1_org, WINDOW_SIZE, POLYNOMIAL_ORDER)

    # 2nd filter run:
    x1_hat2 = savgol_filter(x1_hat, WINDOW_SIZE, POLYNOMIAL_ORDER)

    if FILTERING is True:
        x1      = x1_hat2.copy()  # true copy
        x1_grad = np.gradient(x1_hat2)

else:
    NOISE_PRESENT = False

print("NOISE_PRESENT =", NOISE_PRESENT)

#
# end of noise treatment
#
#############################################


max_x1_grad_no_noise = np.max(x1_grad_no_noise)
max_x1_grad_no_noise_index = np.argmax(x1_grad_no_noise)
max_x1_grad_no_noise_time = t[max_x1_grad_no_noise_index]


####################
#
# construct the tangential line:
max_x1_grad = np.max(x1_grad)
# maximum at what time?
max_x1_grad_index = np.argmax(x1_grad)
max_x1_grad_time = t[max_x1_grad_index]

delta_t = max_x1_grad_time  # [sec]
tangent_time = np.arange(max_x1_grad_time-delta_t, max_x1_grad_time+delta_t, h)

tangent_point = x1[max_x1_grad_index]
tangent_y_min = tangent_point - max_x1_grad * len(tangent_time) /2.0
tangent_y_max = tangent_point + max_x1_grad * len(tangent_time) /2.0
tangent_y_delta = (tangent_y_max - tangent_y_min) / len(tangent_time)
tangent_y    = np.arange(tangent_y_min, tangent_y_max, tangent_y_delta)

# reading the estimated time constants:
if NOISE is True:
    tangent_limit_delta  = 0.005  # robust limit? yes, but only if no noise present
else:
    tangent_limit_delta  = 0.001  # robust limit with noise present?

x1_tangent_zero_ind  = np.argwhere(abs(tangent_y) < tangent_limit_delta)
x1_tangent_63perc_ind = np.argwhere(abs(tangent_y - PERC63) < tangent_limit_delta)
td = t[x1_tangent_zero_ind[0][0]]  # np.float64()
tau = t[x1_tangent_63perc_ind[0][0]] - td # np.float64()

#
# end of construct the tangential line
#
#############################################



plt.suptitle('Process parameter estimation of PT1 process in series (6x)', y = 1.0)
plt.title('6xPT1 process: b0=1.0, a0=1.0, a1=1.0 [sec] (Euler forward)', y = 1.0)

SCALE = 20.0
## plt.plot(t,x1_grad_org*SCALE, label=f'gradient of unfiltered x1 * {SCALE}')


if NOISE is True:

    if FILTERING is True:
        plt.plot(t,x1_org, label="unfiltered measurment")
        plt.plot(t,x1, label=f'6xPT1: step response to\
        \nreference jump to 1.0 at time 0\
        \ndouble Savitzky-Golay filtered x1 with\
        \n  window size {WINDOW_SIZE}, polynomial order {POLYNOMIAL_ORDER}')

        ## plt.plot(t,x1_grad_org*SCALE, label=f'gradient of unfiltered x1 * {SCALE}')

        ## plt.plot(t,x1_grad*SCALE, label=f'gradient of double SG filtered x1 * {SCALE}')

        ## plt.plot(t,x1_grad_no_noise*SCALE, label=f'gradient of x1 without noise * {SCALE}')

        ## plt.plot([],[],label=f'Savitzky-Golay filter (applied 2x):\
        ## \n  windows size = {WINDOW_SIZE}, polynomial order={POLYNOMIAL_ORDER}')

    else:
        plt.plot(t,x1_org, label='6xPT1: step response to\
        \nreference jump to 1.0 at time 0')
        plt.plot(t,x1_grad*SCALE, label=f'gradient of unfiltered x1 * {SCALE}')

else:
    plt.plot(t,x1, label="6xPT1: step response to\
    \nreference jump to 1.0 at time 0")


plt.axhline(PERC63, color='r', label='63% of process value (PV)', linestyle='--')

plt.plot(tangent_time,tangent_y, label='tangential line to maximum gradient of x1', color="black")


plt.axvline(0.0, color='magenta', label='start of bump test', linestyle='-.')
plt.axvline(td, color='magenta', label=f'dead time td≈{td:.2f} [sec]:\
\n  computer reading', linestyle='--')
plt.axvline(tau+td, color='magenta', label=f'time constant tau≈{tau:.2f} [sec]:\
\n  computer reading', linestyle=':')

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


print(f'maximum gradient of potentially noisy measurement x1 = {max_x1_grad:.5f}')
print(f'  at time = {max_x1_grad_time:.5f}')

print(f'maximum gradient of noise-free x1 = {max_x1_grad_no_noise:.5f}')
print(f'  at time = {max_x1_grad_no_noise_time:.5f}')

if NOISE is True:
    print(f'SIGMA = {SIGMA}')

    if FILTERING is True:
        print('Savitzky-Golay filtering (2x):')
        print(f'  polynomial order = {POLYNOMIAL_ORDER} (manually tuned)')
        print(f'  window size = {WINDOW_SIZE} (manually tuned)')


# end of 3.5f6a_6xPT1_step_response_parameter_estimation_for_lambda_tuning.py
