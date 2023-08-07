#!/usr/bin/env python3

# attempt to fit a thrust curve that minimizes the error spread in the
# corresponding alpha vs. drag curve fit.

from lib.constants import gravity
import csv
from math import sqrt
from matplotlib import pyplot as plt
import numpy as np
import numpy as np
from scipy.optimize import curve_fit, least_squares

# [ alpha*r2d, Cl, Cd, qbar, a_flow[0], throttle ]

data = []
with open("params.txt", newline="") as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        data.append(row)

data = np.array(data).astype(float)

print("alpha:", np.min(data[:,0]), np.max(data[:,0]))

def compute_thrust(throttle, vel, xk):
    return xk[0]*sqrt(throttle)
    # return xk[0]*throttle**2 + xk[1]*throttle + xk[2]
    # return pow(throttle, 1/4) * abs(gravity) * scale
    # return xk[0]*vel**2 + xk[1]*vel + xk[2]*throttle**2 + xk[3]*throttle

def compute_drag(accel_total, thrust):
    return thrust - accel_total

def opt_err(xk, plot=False):
    # print("xk:", xk)
    cds = []
    for i in range(len(data)):
        thrust = compute_thrust(data[i,5], sqrt(data[i,3]), xk)
        drag = compute_drag(data[i,4], thrust)
        cd = drag / data[i,3]
        cds.append(cd)
    cds = np.array(cds)
    # print("cds:", cds.shape, cds)

    def func(x, a, b, c):
        return a * x**2 + b * x + c

    # print("data0:", data[:,0].shape)
    cd_popt, pcov = curve_fit(func, data[:,0], cds)

    fits = func(data[:,0], *cd_popt)
    # print("fits:", fits.shape, fits)

    error = func(data[:,0], *cd_popt) - cds
    # print("error:", error)

    if plot:
        plt.plot(data[:,0], cds, ".", label="cd raw data")
        plt.plot(data[:,0], func(data[:,0], *cd_popt), '.', label='fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(cd_popt))
        plt.show()

    return error

# xk = [0, 0, 0, 0, 0]
xk = np.random.rand(5)*2 - 1
res = least_squares(opt_err, xk, verbose=2)
print( res["x"] )
opt_err(res["x"], plot=True)

res["x"][4] = 0
for vel in range(0, 101, 5):
    print("vel: %d " % vel, end='')
    for thr in np.arange(0, 1.01, 0.1):
        f = compute_thrust(thr, vel, res["x"])
        print("%.2f " % f, end='')
    print()