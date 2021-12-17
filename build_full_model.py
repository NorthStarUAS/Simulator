#!/usr/bin/env python3

"""build_full_model

Attempt to use a DMD-esque approach to fit a state transition matrix
that maps previous state to next state, thereby modeling/simulating
flight that closely approximates the original real aircraft.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
import math
from matplotlib import pyplot as plt
import numpy as np
from tqdm import tqdm

from rcUAS_flightdata import flight_loader, flight_interp

from constants import d2r, kt2mps
import quaternion
from system_id import SystemIdentification

# command line arguments
parser = argparse.ArgumentParser(description="nav filter")
parser.add_argument("flight", help="flight data log")
parser.add_argument("--write", required=True, help="write model file name")
args = parser.parse_args()

sysid = SystemIdentification()

# load the flight data
path = args.flight
data, flight_format = flight_loader.load(path)

print("imu records:", len(data["imu"]))
print("gps records:", len(data["gps"]))
if "air" in data:
    print("airdata records:", len(data["air"]))
if "act" in data:
    print("actuator records:", len(data["act"]))
if len(data["imu"]) == 0 and len(data["gps"]) == 0:
    print("not enough data loaded to continue.")
    quit()

# dt estimation
print("Estimating median dt from IMU records:")
iter = flight_interp.IterateGroup(data)
last_time = None
dt_data = []
for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record):
        if "imu" in record:
            imupt = record["imu"]
            if last_time is None:
                last_time = imupt["time"]
            dt_data.append(imupt["time"] - last_time)
            last_time = imupt["time"]
dt_data = np.array(dt_data)
print("IMU mean:", np.mean(dt_data))
print("IMU median:", np.median(dt_data))
imu_dt = float("%.4f" % np.median(dt_data))
print("imu dt:", imu_dt)
            
print("Parsing flight data log:")
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

# iterate through the flight data log, cherry pick the imporant
# parameters that are sensed directly and estimate other important
# parameters as best as we can.

state_names = [ "airspeed**2 (mps)", "throttle",
                "aileron", "elevator", "rudder",
                "phi", "cos(phi)", "the", # gravity vector here?
                "alpha", "beta", "cos(beta)",
                "accel_body[0]", "accel_body[1]", "accel_body[2]",
                "p", "q", "r" ]
sysid.set_state_names(state_names)

for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record) == 0:
        continue
    if "imu" in record:
        imupt = record["imu"]
    if "act" in record:
        actpt = record["act"]
    if "air" in record:
        airpt = record["air"]
        if "pitot_scale" in airpt:
            asi_mps = airpt["airspeed"] * airpt["pitot_scale"] * kt2mps
        else:
            asi_mps = airpt["airspeed"] * kt2mps
    if "filter" in record:
        navpt = record["filter"]
    if "gps" in record:
        gpspt = record["gps"]
        gs_mps = math.sqrt( gpspt["vn"]**2 + gpspt["ve"]**2 )

    # include only "in flight" data
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
    ned2body = quaternion.eul2quat( navpt["phi"], navpt["the"], navpt["psi"] )

    # # ax, ay, az are 'callibrated' but do not include imu bias estimates
    # sensed_accel = np.array( [ imupt["ax"] - navpt["ax_bias"],
    #                            imupt["ay"] - navpt["ay_bias"],
    #                            imupt["az"] - navpt["az_bias"] ] )
    
    # # rotate gravity into body frame and remove it from sensed accelerations
    # g_body = quaternion.transform(ned2body, g)
    # body_accel = sensed_accel - g_body

    # our best estimate of wind velocity in the ned coordinate frame
    if "wind_dir" in airpt:
        wind_psi = 0.5*math.pi - airpt["wind_dir"] * d2r
        wind_mps = airpt["wind_speed"] * kt2mps
        we = math.cos(wind_psi) * wind_mps
        wn = math.sin(wind_psi) * wind_mps
        # smooth abrubt wind changes (usually due to low precision of logging)
        wn_filt = 0.95 * wn_filt + 0.05 * wn
        we_filt = 0.95 * we_filt + 0.05 * we
    else:
        wn_filt = 0
        wn_filt = 0

    # compute ned velocity with wind vector removed
    v_ned = np.array( [navpt["vn"]+wn_filt, navpt["ve"]+we_filt, navpt["vd"]] )
    #print(i, v_ned)

    # rotate ned velocity vector into body frame
    v_body = quaternion.transform(ned2body, v_ned)

    # estimate alpha and beta (requires a decent wind estimate)
    alpha = math.atan2( v_body[2], v_body[0] )
    beta = math.atan2( -v_body[1], v_body[0] )
    #print("v(body):", v_body, "alpha = %.1f" % (alpha/d2r), "beta = %.1f" % (beta/d2r))

    # estimate accelerations in body frame using velocity difference
    # (imu accel biases are too problematic)
    if v_body_last is None:
        v_body_last = v_body.copy()
    accel_body = (v_body - v_body_last) / imu_dt
    v_body_last = v_body.copy()
    
    # build the state vector:
    # airspeed**2 is essentially qbar (without air density)
    # throttle command is our best proxy for thrust
    # cos() and sin() of both phi(roll) and theta(pitch) help
    # linearize around multiple regions.
    # alpha and beta are our best estimate from flight data, it's ok
    # if these are somewhat noisy becasue we are doing a best fit.
    # accel_body are computed from change in ned velocity rotated into the
    # body frame.
    # p, q, r are gyro rates corrected by ekf bias estates.

    # add cos(aileron) and cos(rudder) or abs(aileron & rudder)?
    state = [ asi_mps**2, actpt["throttle"],
              actpt["aileron"], actpt["elevator"], actpt["rudder"],
              navpt["phi"], math.cos(navpt["phi"]), navpt["the"],
              alpha, beta, math.cos(beta),
              accel_body[0], accel_body[1], accel_body[2],
              imupt["p"] - navpt["p_bias"],
              imupt["q"] - navpt["q_bias"],
              imupt["r"] - navpt["r_bias"] ]
    sysid.add_state_vec(state)

states = len(sysid.traindata[0])
print("Number of states:", len(sysid.traindata[0]))
print("Input state vectors:", len(sysid.traindata))

plt.figure()
for j in range(states):
    plt.plot(np.array(sysid.traindata).T[j,:], label="%s" % state_names[j])
plt.legend()
plt.show()

# k > 0 will cascade/stack that many additional previous state vectors
# into the model.  Essentially this means our prediction can be a
# function of "n" (where n = k + 1) previous states.  This could be
# helpful for an integrity monitoring system, but may lead to
# diverging state values in a simulation when most of the states are
# being propagated forward from previous estimates.
k = 0

sysid.fit(k)

sysid.analyze()

sysid.save("idun2_model.json", imu_dt)

# check the fit of the original data versus a selection of
# estimated/propagated states.

if False:
    moving_est = True
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
    for i in range(len(sysid.traindata)):
        v.extend(sysid.traindata[i])
        if moving_est:
            v[-9] = alpha_est
            v[-8] = beta_est
            v[-7] = math.cos(beta_est)
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
            p = sysid.A @ np.array(v)
            #print("p:", p)
            if moving_est:
                alpha_est = p[-9]
                beta_est = p[-8]
                ax_est = p[-6]
                ay_est = p[-5]
                az_est = p[-4]
                p_est = p[-3]
                q_est = p[-2]
                r_est = p[-1]
            pred.append(p)
    Ypred = np.array(pred).T

# airspeed estimator? but need to think through alpha/beta and
# vel_body estimates because all that is interelated, those need to
# also be reestimated as well or the results aren't valid (way over
# optimistic).

# think about using norm(velocity body) to estimate airspeed?
if True:
    pred = []
    asi_est = 0
    alpha_est = 0
    beta_est = 0
    ax_est = 0
    ay_est = 0
    az_est = 0
    v = []
    for i in range(len(sysid.traindata)):
        v.extend(sysid.traindata[i])
        v[-17] = asi_est**2
        v[-9] = alpha_est
        v[-8] = beta_est
        v[-6] = ax_est
        v[-5] = ay_est
        v[-4] = az_est
        v = v[-(k+1)*states:]       # trim old state values if needed
        if len(v) == (k+1)*states:
            #print("A:", A.shape, A)
            #print("v:", np.array(v).shape, np.array(v))
            p = sysid.A @ np.array(v)
            #print("p:", p)
            if p[0] > 0:
                asi_est = math.sqrt(p[0])
            else:
                asi_est = 0
            alpha_est = p[-9]
            beta_est = p[-8]
            ax_est = p[-6]
            ay_est = p[-5]
            az_est = p[-4]
            pred.append(p)
    Ypred = np.array(pred).T
    
for j in range(states):
    plt.figure()
    plt.plot(np.array(sysid.traindata).T[j,k:], label="%s (orig)" % state_names[j])
    plt.plot(Ypred[j,:], label="%s (pred)" % state_names[j])
    plt.legend()
    plt.show()

