#!/usr/bin/env python3

"""do_synth_asi.py

Attempt to use a DMD-esque (FTW) approach to fitting a matrix that
maps previous state to next state.

Author: Curtis L. Olson, University of Minnesota

"""

import argparse
import math
from matplotlib import pyplot as plt
import numpy as np
import os
from tqdm import tqdm
import dask.array as da

from rcUAS_flightdata import flight_loader, flight_interp

# command line arguments
parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('flight', help='flight data log')
parser.add_argument('--gps-lag-sec', type=float, default=0.2,
                    help='gps lag (sec)')
args = parser.parse_args()

# parameters we decided are relevant to airspeed: theta (pitch angle),
# q (pitch rate), az (z axis accel), phi (roll angle), elevator_cmd,
# throttle_cmd, measured airspeed

# constants
r2d = 180.0 / math.pi
d2r = math.pi / 180.0

# load the flight data
path = args.flight
data, flight_format = flight_loader.load(path)

print("imu records:", len(data['imu']))
imu_dt = (data['imu'][-1]['time'] - data['imu'][0]['time']) \
    / float(len(data['imu']))
print("imu dt: %.3f" % imu_dt)
print("gps records:", len(data['gps']))
if 'air' in data:
    print("airdata records:", len(data['air']))
if 'act' in data:
    print("actuator records:", len(data['act']))
if len(data['imu']) == 0 and len(data['gps']) == 0:
    print("not enough data loaded to continue.")
    quit()
    
print("Generating synthetic airspeed model:")
fulldata = []
actpt = {}
airpt = {}
navpt = {}
flying = False
iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record):
        imupt = record['imu']
        if 'act' in record:
            actpt = record['act']
        if 'air' in record:
            airpt = record['air']
        if 'filter' in record:
            navpt = record['filter']
        if 'gps' in record:
            gpspt = record['gps']
            gs_mps = math.sqrt( gpspt['vn']**2 + gpspt['ve']**2 )
            if gs_mps > 10 and airpt['airspeed'] > 15:
                flying = True
            else:
                flying = False
        if not flying:
            continue
        if 'time' in actpt and 'time' in navpt and 'time' in airpt:
            state = [ airpt['airspeed'], actpt['throttle'],
                      actpt['aileron'], actpt['elevator'], actpt['rudder'],
                      math.sin(navpt['phi']), math.cos(navpt['the']),
                      imupt['p'], imupt['q'], imupt['r'] ]
            fulldata.append(state)

states = len(fulldata[0])
print("Number of states:", len(fulldata[0]))
print("Input state vectors:", len(fulldata))

#traindata = fulldata[:10000]
traindata = fulldata

data1 = []
k = 0                           # add this many previous states
for i in range(k, len(traindata)):
    v = list(traindata[i])
    for j in range(1, k+1):
        v.extend(traindata[i-j])
    #print(v)
    data1.append(v)
    
X = np.array(data1[:-1]).T
Y = np.array(traindata[1+k:]).T
print("X:\n", X.shape, np.array(X))
print("Y:\n", Y.shape, np.array(Y))

# Y = A * X, solve for A, now A is a matrix that projects v_n-1 -> v_n(est)

# X isn't nxn and doesn't have a direct inverse, so first perform an svd:
#
# Y = A * U * D * V.T

print("dask svd...")
daX = da.from_array(X, chunks=(X.shape[0], 10000)).persist()
u, s, vh = da.linalg.svd(daX)
print("u:\n", u.shape, u)
print("s:\n", s.shape, s)
print("vh:\n", vh.shape, vh)
Xr = (u * s) @ vh[:states*(k+1), :]
#print("Xr:\n", Xr.compute())
#print( "dask close?", np.allclose(X, Xr.compute()) )

# print("numpy svd...")
# (u, s, vh) = np.linalg.svd(X, full_matrices=True)
# print("u:\n", u.shape, u)
# print("s:\n", s.shape, s)
# print("vh:\n", vh.shape, vh)
# print( "close?", np.allclose(X, ((u * s) @ vh[:states*(k+1), :])) )

# after algebraic manipulation
#
# A = Y * V * D.inv() * U.T

v = vh.T
print("s inv:", (1/s).compute() )

#tmp1 = v[:,:states*(k+1)] * (1/s)
#tmp2 = tmp1 @ u.T
#tmp3 = Y @ tmp2
A = (Y @ (v[:,:states*(k+1)] * (1/s)) @ u.T).compute()
print("A rank:", np.linalg.matrix_rank(A))
print("A:\n", A.shape, A)

plt.figure()
for j in range(states):
    plt.plot(X[j,:], label="%d" % j)
plt.legend()
plt.show()

pred = []
p_est = 0
q_est = 0
r_est = 0
v = []
for i in range(len(fulldata)):
    v.extend(fulldata[i])
    v[-3] = p_est
    v[-2] = q_est
    v[-1] = r_est
    v = v[-(k+1)*states:]       # trim
    if len(v) == (k+1)*states:
        #print("A:", A.shape, A)
        #print("v:", np.array(v).shape, np.array(v))
        p = A @ np.array(v)
        #print("p:", p)
        p_est = p[-3]
        q_est = p[-2]
        r_est = p[-1]
        pred.append(p)
Ypred = np.array(pred).T

for j in range(states):
    plt.figure()
    plt.plot(np.array(fulldata).T[j,k:], label="orig %d" % j)
    plt.plot(Ypred[j,:], label="pred %d" % j)
    plt.legend()
    plt.show()

quit()
