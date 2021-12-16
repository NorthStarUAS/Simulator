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
import dask.array as da         # dnf install python3-dask+array
import json

from rcUAS_flightdata import flight_loader, flight_interp

import quaternion
from constants import d2r, kt2mps

# command line arguments
parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('flight', help='flight data log')
args = parser.parse_args()

# parameters we decided are relevant to airspeed: theta (pitch angle),
# q (pitch rate), az (z axis accel), phi (roll angle), elevator_cmd,
# throttle_cmd, measured airspeed

# load the flight data
path = args.flight
data, flight_format = flight_loader.load(path)

print("imu records:", len(data['imu']))
print("gps records:", len(data['gps']))
if 'air' in data:
    print("airdata records:", len(data['air']))
if 'act' in data:
    print("actuator records:", len(data['act']))
if len(data['imu']) == 0 and len(data['gps']) == 0:
    print("not enough data loaded to continue.")
    quit()

# dt estimation
print("estimating dt from IMU records..")
iter = flight_interp.IterateGroup(data)
last_time = None
dt_data = []
for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record):
        if 'imu' in record:
            imupt = record['imu']
            if last_time is None:
                last_time = imupt['time']
            dt_data.append(imupt['time'] - last_time)
            last_time = imupt['time']
dt_data = np.array(dt_data)
print("IMU mean:", np.mean(dt_data))
print("IMU median:", np.median(dt_data))
imu_dt = float("%.4f" % np.median(dt_data))
print("imu dt:", imu_dt)
            
print("Generating synthetic flight dynamics model:")
fulldata = []
actpt = {}
airpt = {}
navpt = {}
flying = False
iter = flight_interp.IterateGroup(data)
g = np.array( [ 0, 0, -9.81 ] )
gs_mps = 0
asi_mps = 0
v_body_last = None
wn_filt = 0
we_filt = 0

# iterate through the flight data log
for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record) == 0:
        continue
    if 'imu' in record:
        imupt = record['imu']
        #print(imupt)
    if 'act' in record:
        actpt = record['act']
    if 'air' in record:
        airpt = record['air']
        if 'pitot_scale' in airpt:
            asi_mps = airpt['airspeed'] * airpt['pitot_scale'] * kt2mps
        else:
            asi_mps = airpt['airspeed'] * kt2mps
        #print(airpt)
    if 'filter' in record:
        navpt = record['filter']
        #print(navpt)
    if 'gps' in record:
        gpspt = record['gps']
        gs_mps = math.sqrt( gpspt['vn']**2 + gpspt['ve']**2 )

    # simulation is only modeled for "in flight" conditions
    if not flying and gs_mps > 10 and asi_mps > 7:
        print("Start flying @", i)
        flying = True
    elif flying and gs_mps < 5 and asi_mps < 3:
        print("Stop flying @", i)
        flying = False
        v_body_last = None
    if not flying:
        continue

    # transformation between NED coordations and body coordinates
    ned2body = quaternion.eul2quat( navpt['phi'], navpt['the'], navpt['psi'] )

    # # ax, ay, az are 'callibrated' but do not include imu bias estimates
    # sensed_accel = np.array( [ imupt['ax'] - navpt['ax_bias'],
    #                            imupt['ay'] - navpt['ay_bias'],
    #                            imupt['az'] - navpt['az_bias'] ] )
    
    # # rotate gravity into body frame and remove it from sensed accelerations
    # g_body = quaternion.transform(ned2body, g)
    # body_accel = sensed_accel - g_body

    # our best estimate of wind velocity in the ned coordinate frame
    if 'wind_dir' in airpt:
        wind_psi = 0.5*math.pi - airpt['wind_dir'] * d2r
        wind_mps = airpt['wind_speed'] * kt2mps
        we = math.cos(wind_psi) * wind_mps
        wn = math.sin(wind_psi) * wind_mps
        # smooth abrubt wind changes (usually due to low precision of logging)
        wn_filt = 0.95 * wn_filt + 0.05 * wn
        we_filt = 0.95 * we_filt + 0.05 * we
    else:
        wn_filt = 0
        wn_filt = 0

    # ned velocity with wind effects removed
    v_ned = np.array( [ navpt['vn'] + wn_filt,
                        navpt['ve'] + we_filt,
                        navpt['vd'] ] )
    #print(i, v_ned)

    # velocity in body frame
    v_body = quaternion.transform(ned2body, v_ned)

    # alpha/beta estimates
    alpha = math.atan2( v_body[2], v_body[0] )
    beta = math.atan2( -v_body[1], v_body[0] )
    #print("v(body):", v_body, "alpha = %.1f" % (alpha/d2r), "beta = %.1f" % (beta/d2r))

    # estimate accelerations in body frame (imu accels may be too
    # biased to be useful)
    if v_body_last is None:
        v_body_last = v_body.copy()
    accel_body = (v_body - v_body_last) / imu_dt
    v_body_last = v_body.copy()
    
    # airspeed and throttle values are proxies for qbar, alpha, thrust
    state = [ asi_mps**2, actpt['throttle'],
              actpt['aileron'], actpt['elevator'], actpt['rudder'],
              math.cos(navpt['phi']), math.cos(navpt['the']),
              math.sin(navpt['phi']), math.sin(navpt['the']),
              alpha, beta,
              accel_body[0], accel_body[1], accel_body[2],
              imupt['p'] - navpt['p_bias'],
              imupt['q'] - navpt['q_bias'],
              imupt['r'] - navpt['r_bias'] ]
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

moving_est = False

pred = []
alpha_est = 0
beta_est = 0
ax_est = 0
ay_est = 0
az_est = 0
p_est = 0
q_est = 0
r_est = 0
v = []
for i in range(len(fulldata)):
    v.extend(fulldata[i])
    if moving_est:
        v[-8] = alpha_est
        v[-7] = beta_est
        v[-6] = ax_est
        v[-5] = ay_est
        v[-4] = az_est
        v[-3] = p_est
        v[-2] = q_est
        v[-1] = r_est
    v = v[-(k+1)*states:]       # trim old state values if needed
    if len(v) == (k+1)*states:
        #print("A:", A.shape, A)
        #print("v:", np.array(v).shape, np.array(v))
        p = A @ np.array(v)
        #print("p:", p)
        if moving_est:
            alpha_est = p[-8]
            beta_est = p[-7]
            ax_est = p[-6]
            ay_est = p[-5]
            az_est = p[-4]
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

model = {
    "dt": imu_dt,
    "A": A.tolist()
}

f = open("skywalker_model.json", "w")
json.dump(model, f, indent=4)
f.close()
