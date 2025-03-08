# -*- coding: utf-8 -*-
"""
2025-03-07/08

step_response_parameter_estimation_for_lambda_tuning - experiments.py


Based on program:
    step_response_parameter_estimation_for_lambda_tuning.py


tests: OK!


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
# denominator polynomial: (a1*s + a0)*(a1*s + a0)*(a1*s + a0)


# parameters for Lambda Tuning:
tau=2.4  # [sec]; just some ini value for now
td=0.8  # [sec]; just some ini value for now
gp=1.0   # everything stays within a unit jump
PERC63 = 0.63  # 63% of total PV change


STEPS = 1250  # 1250 for process step responses; 750 for filter tests
tstop = STEPS * h + tstart
t = np.arange(tstart,tstop,h)


x1       = np.zeros([STEPS])  # clean measurement = clean process output
x1_noisy = np.zeros([STEPS])  # noisy measurement = noisy process output
# noise has no memory (--> Markov process) and thus the simulation of a noisy measurement
# needs an extra array

u  = np.zeros([STEPS])  # process input signal
u[:] = 1.0
u[0] = 0.0  # here, process input jump at second point in time


NOISE = True
FILTERING = True
# SIGMA = 0.0035  # sigma of noisy measurement
SIGMA = 0.01  # sigma of noisy measurement
x1_noise = 1.0 + np.random.normal(0.0, SIGMA, STEPS)  # multiplicative <--> additive??


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

        x1[k+1] = 1.0/a1**3.0 * ((a1**3.0 - A0)*x1[k-2] \
                                     - 3.0*a1**2.0*(a0*h - a1)*x1[k] \
                                     - 3.0*a1*A1*x1[k-1]
                                     + b0*h**3.0*u[k-2])

        if NOISE is True:
            x1_noisy[k+1] = x1[k+1] * x1_noise[k+1]


if NOISE is True:
    x1_org = x1_noisy.copy()  # true copy

else:
    x1_org = x1.copy()  # true copy


# calculating the gradient of the measurement:
x1_grad = np.gradient(x1_org)
x1_grad_no_noise = np.gradient(x1)
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


#############################################
#
# SG: search for a good window size for one filtering run
# 2025-03-08: this kind of brutal search does not really help here, since
#   very high window sizes almost always win, even after 10000 test runs for
#   a specific window size! (with polynomial order = 1 or even 2 or 3)
#   Very high window sizes makes the filter output sluggish.
"""
ws = np.arange(11,161,10)  # test a range of window sizes
x1_test_time_ws =  [0.0 for i in ws]
WS_TESTS = 10000
time_i = 0
for ws_i in ws:
    x1_test_time = []
    for i in range(0,WS_TESTS):
        x1_sg = savgol_filter(x1_org, ws_i, 1)  # polynomial order seems to be OK
        x1_grad_t = np.gradient(x1_sg)
        max_x1_grad_index_t = np.argmax(x1_grad_t)
        max_x1_grad_time_t = t[max_x1_grad_index_t]
        x1_test_time.append(max_x1_grad_time_t)

    x1_test_time_ws[time_i] = np.mean(x1_test_time)
    time_i += 1

x1_test_time_min = np.empty(len(x1_test_time_ws))
x1_test_time_min = abs(x1_test_time_ws - max_x1_grad_no_noise_time)
good_ws = ws[np.argmin(x1_test_time_min)]

print(f'\nA good window size for a Savitzky-Golay filter\
 based on {WS_TESTS} test runs is {good_ws}\n')
"""
#
#############################################



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
## td = t[x1_tangent_zero_ind[0][0]]  # np.float64()
## tau = t[x1_tangent_63perc_ind[0][0]] - td # np.float64()

#
# end of construct the tangential line
#
#############################################



plt.suptitle('Process parameter estimation of PT1 process in series (3x)', y = 1.0)
plt.title('3xPT1 process: b0=1.0, a0=1.0, a1=1.0 [sec] (Euler forward)', y = 1.0)

SCALE = 20.0
## plt.plot(t,x1_grad_org*SCALE, label=f'gradient of unfiltered x1 * {SCALE}')


if NOISE is True:

    if FILTERING is True:
        plt.plot(t,x1_org, label="unfiltered measurment")
        plt.plot(t,x1, label=f'3xPT1: step response to\
        \nreference jump to 1.0 at time 0\
        \ntd≈0.8sec, tau≈2.4sec: manual reading\
        \ndouble Savitzky-Golay filtered x1 with\
        \n  window size {WINDOW_SIZE}, polynomial order {POLYNOMIAL_ORDER}')

        ## plt.plot(t,x1_grad_org*SCALE, label=f'gradient of unfiltered x1 * {SCALE}')

        ## plt.plot(t,x1_grad*SCALE, label=f'gradient of double SG filtered x1 * {SCALE}')

        ## plt.plot(t,x1_grad_no_noise*SCALE, label=f'gradient of x1 without noise * {SCALE}')

        ## plt.plot([],[],label=f'Savitzky-Golay filter (applied 2x):\
        ## \n  windows size = {WINDOW_SIZE}, polynomial order={POLYNOMIAL_ORDER}')

    else:
        plt.plot(t,x1_noisy, label='3xPT1: step response to\
        \nreference jump to 1.0 at time 0\
        \ntd≈0.8sec, tau≈2.4sec: manual reading')
        plt.plot(t,x1_grad*SCALE, label=f'gradient of unfiltered x1 * {SCALE}')

else:
    plt.plot(t,x1, label="3xPT1: step response to\
    \nreference jump to 1.0 at time 0\
    \ntd≈0.8sec, tau≈2.4sec: manual reading")


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


# end of step_response_parameter_estimation_for_lambda_tuning - experiments.py
