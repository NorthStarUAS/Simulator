#!/usr/bin/env python3

"""build_full_model

Attempt to use a DMD-esque approach to fit a state transition matrix
that maps previous state to next state, thereby modeling/simulating
flight that closely approximates the original real aircraft.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
from math import cos, pi, sin
from matplotlib import pyplot as plt
import numpy as np
from tqdm import tqdm

from rcUAS_flightdata import flight_loader, flight_interp

from lib.constants import d2r, kt2mps
from lib.system_id import SystemIdentification

# command line arguments
parser = argparse.ArgumentParser(description="nav filter")
parser.add_argument("flight", help="flight data log")
parser.add_argument("--write", required=True, help="write model file name")
args = parser.parse_args()

sysid = SystemIdentification()

independent_states = [
    "sqrt(throttle)",           # a very approximate throttle vs. thrust curve
    "aileron", "abs(aileron)",
    "elevator",
    "rudder", "abs(rudder)",    # flight controls (* qbar)
    "bgx", "bgy", "bgz"         # gravity rotated into body frame
]

dependent_states = [
    "bvx", "bvy", "bvz",        # body frame velocities (* qbar)
    "p", "q", "r"               # body rates
]

state_names = independent_states + dependent_states
sysid.state_mgr.set_state_names(independent_states, dependent_states)

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

sysid.state_mgr.set_dt(imu_dt)
            
print("Parsing flight data log:")
actpt = {}
airpt = {}
navpt = {}
g = np.array( [ 0, 0, -9.81 ] )

# iterate through the flight data log, cherry pick the selected parameters
iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record) == 0:
        continue
    if "imu" in record:
        imupt = record["imu"]
        sysid.state_mgr.set_time( imupt["time"] )
        p = imupt["p"]
        q = imupt["q"]
        r = imupt["r"]
        if "p_bias" in navpt:
            p -= navpt["p_bias"]
            q -= navpt["q_bias"]
            r -= navpt["r_bias"]
        sysid.state_mgr.set_gyros(p, q, r)
    if "act" in record:
        actpt = record["act"]
        sysid.state_mgr.set_throttle( actpt["throttle"] )
        sysid.state_mgr.set_flight_surfaces( actpt["aileron"], actpt["elevator"],
                                       actpt["rudder"] )
    if "air" in record:
        airpt = record["air"]
        asi_mps = airpt["airspeed"] * kt2mps
        # add in correction factor if available
        if "pitot_scale" in airpt:
            asi_mps *= airpt["pitot_scale"]
        sysid.state_mgr.set_airdata( asi_mps )
        if "wind_dir" in airpt:
            wind_psi = 0.5 * pi - airpt["wind_dir"] * d2r
            wind_mps = airpt["wind_speed"] * kt2mps
            we = cos(wind_psi) * wind_mps
            wn = sin(wind_psi) * wind_mps
            sysid.state_mgr.set_wind(wn, we)
    if "filter" in record:
        navpt = record["filter"]
        sysid.state_mgr.set_orientation( navpt["phi"], navpt["the"], navpt["psi"] )
        sysid.state_mgr.set_ned_velocity( navpt["vn"], navpt["ve"], navpt["vd"] )
    if "gps" in record:
        gpspt = record["gps"]

    if sysid.state_mgr.is_flying():
        sysid.state_mgr.compute_body_frame_values(body_vel=True)
        state = sysid.state_mgr.gen_state_vector()
        #print(sysid.state_mgr.state2dict(state))
        sysid.add_state_vec(state)

states = len(sysid.traindata[0])
print("Number of states:", len(sysid.traindata[0]))
print("Input state vectors:", len(sysid.traindata))

# plt.figure()
# for j in range(states):
#     print(j, state_names[j])
#     plt.plot(np.array(sysid.traindata).T[j,:], label="%s" % state_names[j])
# plt.legend()
# plt.show()

# k > 0 will cascade/stack that many additional previous state vectors
# into the model.  Essentially this means our prediction can be a
# function of "n" (where n = k + 1) previous states.  This could be
# helpful for an integrity monitoring system, but may lead to
# diverging state values in a simulation when most of the states are
# being propagated forward from previous estimates.
sysid.fit()
sysid.analyze()
sysid.save(args.write, imu_dt)

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
            v[-7] = abs(beta_est)
            v[-6] = ax_est
            v[-5] = ay_est
            v[-4] = az_est
            v[-3] = p_est
            v[-2] = q_est
            v[-1] = r_est
        v = v[-states:]       # trim old state values if needed
        if len(v) == states:
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

if True:
    est_index_list = sysid.state_mgr.get_state_index( dependent_states )
    #print("est_index list:", est_index_list)
    est_val = [0.0] * len(dependent_states)
    pred = []
    v = []
    for i in range(len(sysid.traindata)):
        v.extend(sysid.traindata[i])
        v = v[-states:]       # trim old state values if needed
        for j, index in enumerate(est_index_list):
            v[index-states] = est_val[j]
        if len(v) == states:
            #print("A:", A.shape, A)
            #print("v:", np.array(v).shape, np.array(v))
            p = sysid.A @ np.array(v)
            #print("p:", p)
            for j, index in enumerate(est_index_list):
                est_val[j] = p[index-states]
                param = sysid.model["parameters"][index]
                min = param["min"]
                max = param["max"]
                med = param["median"]
                std = param["std"]
                #if est_val[j] < med - 2*std: est_val[j] = med - 2*std
                #if est_val[j] > med + 2*std: est_val[j] = med + 2*std
                if est_val[j] < min: est_val[j] = min
                if est_val[j] > max: est_val[j] = max
            pred.append(p)
    Ypred = np.array(pred).T
    
    index_list = sysid.state_mgr.get_state_index( dependent_states )
    for j in index_list:
        plt.figure()
        plt.plot(np.array(sysid.traindata).T[j,:], label="%s (orig)" % state_names[j])
        plt.plot(Ypred[j,:], label="%s (pred)" % state_names[j])
        plt.legend()
        plt.show()

