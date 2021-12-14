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

d2r = math.pi / 180.0
kt2mps = 0.5144444444444444444
mps2kt = 1.0 / kt2mps

# command line arguments
parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('flight', help='flight data log')
parser.add_argument('--gps-lag-sec', type=float, default=0.2,
                    help='gps lag (sec)')
args = parser.parse_args()

# parameters we decided are relevant to airspeed: theta (pitch angle),
# q (pitch rate), az (z axis accel), phi (roll angle), elevator_cmd,
# throttle_cmd, measured airspeed

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

# computes a quaternion from the given euler angles
def eul2quat(phi_rad, the_rad, psi_rad):
    sin_psi = math.sin(psi_rad * 0.5)
    cos_psi = math.cos(psi_rad * 0.5)
    sin_the = math.sin(the_rad * 0.5)
    cos_the = math.cos(the_rad * 0.5)
    sin_phi = math.sin(phi_rad * 0.5)
    cos_phi = math.cos(phi_rad * 0.5)

    q = np.zeros(4)
    q[0] = cos_psi*cos_the*cos_phi + sin_psi*sin_the*sin_phi  
    q[1] = cos_psi*cos_the*sin_phi - sin_psi*sin_the*cos_phi
    q[2] = cos_psi*sin_the*cos_phi + sin_psi*cos_the*sin_phi  
    q[3] = sin_psi*cos_the*cos_phi - cos_psi*sin_the*sin_phi
    
    return q

def quaternion_real(quaternion):
    """Return real part of quaternion.

    >>> quaternion_real([3, 0, 1, 2])
    3.0

    """
    return float(quaternion[0])

def quaternion_imag(quaternion):
    """Return imaginary part of quaternion.

    >>> quaternion_imag([3, 0, 1, 2])
    array([ 0.,  1.,  2.])

    """
    return np.array(quaternion[1:4], dtype=np.float64, copy=True)

def quaternion_transform(quat, v):
    # Transform a vector from the current coordinate frame to a coordinate
    # frame rotated with the quaternion
    r = 2.0 / np.dot(quat, quat)
    qimag = quaternion_imag(quat)
    qr = quaternion_real(quat)
    tmp1 = (r*qr*qr - 1.0)*np.array(v, dtype=np.float64, copy=True)
    tmp2 = (r*np.dot(qimag, v))*qimag
    tmp3 = (r*qr)*np.cross(qimag, v)
    return tmp1 + tmp2 - tmp3

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
v_body_last = np.array( [0.0, 0.0, 0.0] )

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
        asi_mps = airpt['airspeed'] * airpt['pitot_scale'] * kt2mps
        #print(airpt)
    if 'filter' in record:
        navpt = record['filter']
        #print(navpt)
    if 'gps' in record:
        gpspt = record['gps']
        gs_mps = math.sqrt( gpspt['vn']**2 + gpspt['ve']**2 )

    # simulation is only modeled for "in flight" conditions
    if gs_mps > 10 and asi_mps > 7:
        flying = True
    elif gs_mps < 5 and asi_mps < 3:
        flying = False
    if not flying:
        continue

    # transformation between NED coordations and body coordinates
    ned2body = eul2quat( navpt['phi'], navpt['the'], navpt['psi'] )

    # # ax, ay, az are 'callibrated' but do not include imu bias estimates
    # sensed_accel = np.array( [ imupt['ax'] - navpt['ax_bias'],
    #                            imupt['ay'] - navpt['ay_bias'],
    #                            imupt['az'] - navpt['az_bias'] ] )
    
    # # rotate gravity into body frame and remove it from sensed accelerations
    # g_body = quaternion_transform(ned2body, g)
    # body_accel = sensed_accel - g_body

    # our best estimate of wind velocity in the ned coordinate frame
    wind_psi = 0.5*math.pi - airpt['wind_dir'] * d2r
    wind_mps = airpt['wind_speed'] * kt2mps
    we = math.cos(wind_psi) * wind_mps
    wn = math.sin(wind_psi) * wind_mps

    # ned velocity with wind effects removed
    v_ned = np.array( [ navpt['vn'] + wn, navpt['ve'] + we, navpt['vd'] ] )

    # velocity in body frame
    v_body = quaternion_transform(ned2body, v_ned)

    # estimate accelerations in body frame (imu accels may be too
    # biased to be useful)
    accel_body = (v_body - v_body_last) / imu_dt
    v_body_last = v_body.copy()
    
    # alpha/beta estimates
    alpha = math.atan2( v_body[2], v_body[0] )
    beta = math.atan2( v_body[1], v_body[0] )
    #print("v(body):", v_body, "alpha = %.1f" % (alpha/d2r), "beta = %.1f" % (beta/d2r))

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

f = open("skywalker_model.json", "w")
json.dump(A.tolist(), f, indent=4)
f.close()
