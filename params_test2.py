#!/usr/bin/env python3

from lib.constants import gravity
import csv
from math import sqrt
from matplotlib import pyplot as plt
import numpy as np
import numpy as np
from scipy.optimize import curve_fit, least_squares

# [ alpha*r2d, Cl, Cd, qbar, a_flow[0], throttle ]
# [ alpha*r2d, Cl, qbar, ax, a_3body[0], throttle ]
data = []
with open("params.txt", newline="") as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        data.append(row)

data = np.array(data).astype(float)

alphas = data[:,0]
Cls = data[:,1]
Cds = data[:,2]
qbars = data[:,3]
axs = data[:,4]
throttles = data[:,5]

# initial thrust model
thrusts = []
for i in range(len(throttles)):
    thrust = 0.75 * sqrt(throttles[i]) * abs(gravity)
    thrusts.append(thrust)
thrusts = np.array(thrusts)

print("alpha:", np.min(alphas), np.max(alphas))
print("vel:", sqrt(2*np.max(qbars)))

# def compute_thrust(throttle, vel, xk):
#     # return xk[0]*sqrt(throttle)
#     return xk[0] * sqrt(throttle) * abs(gravity)
#     # return xk[0]*throttle**2 + xk[1]*throttle + xk[2]
#     # return pow(throttle, 1/4) * abs(gravity) * scale
#     # return xk[0]*vel**2 + xk[1]*vel + xk[2]*throttle**2 + xk[3]*throttle

def compute_drag(accel_total, thrust):
    return thrust - accel_total

def func(x, a, b, c):
    return a * x**2 + b * x + c

while True:
    # using a basic thrust function, compute drag and fit a drag vs. alpha
    # function

    cds = []
    for i in range(len(data)):
        # thrust = compute_thrust(throttles[i], sqrt(qbars[i]), [0.75])
        drag = thrusts[i] - axs[i]
        cd = drag / qbars[i]
        print("ax:", axs[i], "drag:", drag, "qbar:", qbars[i], "cd:", cd)
        cds.append(cd)
    cds = np.array(cds)
    # print("cds:", cds.shape, cds)

    # let's look at the drag curves by airspeed range
    plt.figure()
    step = 5
    for s in range(0, 120, step):
        vals = []
        for i in range(len(data)):
            vel = sqrt(2*qbars[i])
            if vel >= s-(step*0.5) and vel <= s+(step*0.5):
                vals.append( [ alphas[i], cds[i] ] )
        vals = np.array(vals)
        if ( len(vals)):
            print("vals:", vals.shape, vals)
            # plt.plot(vals[:,0], vals[:,1], ".", label="%d mps" % s)
            popt, pcov = curve_fit(func, vals[:,0], vals[:,1])
            plt.plot(vals[:,0], func(vals[:,0], *popt), ".", label="%d mps" % s)
    plt.xlabel("Alpha (deg)")
    plt.ylabel("Coefficient of Drag")
    plt.legend()
    plt.show()

    # print("data0:", data[:,0].shape)
    cd_popt, pcov = curve_fit(func, alphas, cds)
    # fits = func(data[:,0], *cd_popt)
    # # print("fits:", fits.shape, fits)

    # error = func(data[:,0], *cd_popt) - cds
    # # print("error:", error)

    plt.figure()
    plt.plot(alphas, cds, ".", label="cd raw data")
    plt.plot(alphas, func(alphas, *cd_popt), '.', label='fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(cd_popt))
    plt.xlabel("Alpha (deg)")
    plt.ylabel("Coefficient of drag")
    plt.show()

    # using the alpha vs. drag function, generate new thrust data points
    for i in range(len(data)):
        cd = func(alphas[i], *cd_popt)
        drag = cd * qbars[i]
        thrusts[i] = drag + axs[i]  # seperate out gravity from thrust
        # thrusts[i] = cd + axs[i]

    # let's look at the thrust curves by airspeed range
    plt.figure()
    step = 5
    for s in range(0, 100, step):
        vals = []
        for i in range(len(data)):
            vel = sqrt(qbars[i])
            if vel >= s-(step*0.5) and vel <= s+(step*0.5):
                vals.append( [ throttles[i], thrusts[i] ] )
        vals = np.array(vals)
        if ( len(vals)):
            print("vals:", vals.shape, vals)
            # plt.plot(vals[:,0], vals[:,1], ".", label="%d mps" % s)
            popt, pcov = curve_fit(func, vals[:,0], vals[:,1])
            plt.plot(vals[:,0], func(vals[:,0], *popt), ".", label="%d mps" % s)
    plt.xlabel("Throttle position (normalized)")
    plt.ylabel("Thrust Force/Acceleration (mps^2)")
    plt.legend()
    plt.show()

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