#!/usr/bin/env python3

"""build_full_model

Attempt to use a DMD-esque approach to fit a state transition matrix
that maps previous state to next state, thereby modeling/simulating
flight that closely approximates the original real aircraft.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
import json
from math import cos, pi, sin
from matplotlib import pyplot as plt
import numpy as np
from tqdm import tqdm

from flightdata import flight_loader, flight_interp

from lib.constants import d2r, r2d, kt2mps
from lib.state_mgr import StateManager
from lib.system_id import SystemIdentification
from lib.wind import Wind

# command line arguments
parser = argparse.ArgumentParser(description="build full model")
parser.add_argument("flight", help="flight data log")
parser.add_argument("--write", required=True, help="write model file name")
parser.add_argument("--vehicle", default="wing", choices=["wing", "quad"], help="vehicle type represented by data file")
parser.add_argument("--invert-elevator", action='store_true', help="invert direction of elevator")
parser.add_argument("--invert-rudder", action='store_true', help="invert direction of rudder")
args = parser.parse_args()

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

state_mgr = StateManager(args.vehicle)

if flight_format == "cirrus_csv":
    # question 1: seem to get a better flaps up fit to airspeed (vs. qbar) but fails to converge for 50% flaps
    # qbar only converges for both conditions
    # question 2: would it be useful to have a gamma (flight path angle) parameter (may help asi)

    # flight controls
    inceptor_states = [
        "aileron",
        "elevator",
        "rudder",
        "throttle",
    ]

    # sensors (directly sensed, or directly converted)
    direct_states = [
        "p", "q", "r",        # imu (body) rates
        "ax",                 # thrust - drag
        "ay",                 # side force
        "az",                 # lift
        "bgx", "bgy", "bgz",  # gravity rotated into body frame
        "alpha_dot",
        "alpha_deg",          # angle of attack
        "beta_deg",           # side slip angle
        "airspeed_mps",
    ]

    # terms (combinations of states)
    terms_list = [
        "qbar",
        "aileron*qbar",
        "abs(aileron)*qbar",
        "elevator*qbar",
        "abs(elevator)*qbar",
        "rudder*qbar",
        "abs(rudder)*qbar",
        "alpha_dot_term2",
        "alpha_dot_term3",
        # state history can improve fit and output parameter prediction, but reduce determinism
        "sin(alpha_prev1)*qbar", "sin(beta_prev1)*qbar",
        "ax_prev1", "ay_prev1", "az_prev1",
        "p_prev1", "q_prev1", "r_prev1",
        "abs(ay)", "abs(bgy)",
    ]

    # states to predict
    output_states = [
        "p",
    ]

    # bins of unique flight conditions
    conditions = [
        { "flaps": 0 },
        { "flaps": 0.5 }
    ]

train_states = inceptor_states + direct_states + terms_list
state_names = inceptor_states + direct_states + output_states
state_mgr.set_state_names(inceptor_states, direct_states, output_states)

if flight_format == "cirrus_csv":
    state_mgr.set_is_flying_thresholds(75*kt2mps, 65*kt2mps)

# dt estimation
print("Estimating median dt from IMU records:")
iter = flight_interp.IterateGroup(data)
last_time = None
dt_data = []
max_airspeed = 0
for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record):
        if "imu" in record:
            imupt = record["imu"]
            if last_time is None:
                last_time = imupt["time"]
            dt_data.append(imupt["time"] - last_time)
            last_time = imupt["time"]
        if "air" in record:
            airpt = record["air"]
            if airpt["airspeed"] > max_airspeed:
                max_airspeed = airpt["airspeed"]
dt_data = np.array(dt_data)
print("IMU mean:", np.mean(dt_data))
print("IMU median:", np.median(dt_data))
imu_dt = float("%.4f" % np.median(dt_data))
print("imu dt:", imu_dt)
print("max airspeed in flight:", max_airspeed )

state_mgr.set_dt(imu_dt)

print("Parsing flight data log:")
actpt = {}
airpt = {}
navpt = {}
g = np.array( [ 0, 0, -9.81 ] )

wn = 0
we = 0
wd = 0

pitot_scale = None
psi_bias = None
wn_interp = None
we_interp = None

# backup wind estimator if needed
windest = Wind()

if False or flight_format != "cirrus_csv":
    # note: wind estimates are only needed for estimating alpha/beta (or
    # bvz/bvy) which is not needed if the aircraft is instrumented with
    # alpha/beta vanes and these are measured directly.

    from lib.wind2 import Wind2
    w2 = Wind2()
    pitot_scale, psi_bias, wn_interp, we_interp = w2.estimate( flight_interp.IterateGroup(data), imu_dt )

# condition data collectors
cond_list = []
for i in range(len(conditions)):
    cond_list.append( { "traindata_list": [], "coeff": [] } )

# for devel iteration, limit the number of records processed to speed things up
max_count = 75000

# iterate through the flight data log (a sequence of time samples of all the measured states)
iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    if max_count > 0 and i > max_count:
        break
    record = iter.next()
    if len(record) == 0:
        continue

    # 1. Do the messy work of cherry picking out the direct measured states from each time sample
    if "filter" in record:
        # need ahead of air in case we are doing a wind estimate
        navpt = record["filter"]
    else:
        continue
    if "imu" in record:
        imupt = record["imu"]
        state_mgr.set_time( imupt["time"] )
        p = imupt["p"]
        q = imupt["q"]
        r = imupt["r"]
        if "p_bias" in navpt:
            p -= navpt["p_bias"]
            q -= navpt["q_bias"]
            r -= navpt["r_bias"]
        state_mgr.set_gyros( np.array([p, q, r]) )
        ax = imupt["ax"]
        ay = imupt["ay"]
        az = imupt["az"]
        if "ax_bias" in navpt:
            ax -= navpt["ax_bias"]
            ay -= navpt["ay_bias"]
            az -= navpt["az_bias"]
        state_mgr.set_accels( np.array([ax, ay, az]) )
    if "act" in record:
        actpt = record["act"]
        if args.vehicle == "wing":
            state_mgr.set_throttle( actpt["throttle"] )
            ail = actpt["aileron"]
            ele = actpt["elevator"]
            rud = actpt["rudder"]
            if args.invert_elevator:
                ele = -ele
            if args.invert_rudder:
                rud = -rud
            if "flaps" in actpt:
                flaps = actpt["flaps"]
            else:
                flaps = 0
            state_mgr.set_flight_surfaces( ail, ele, rud, flaps )
        elif args.vehicle == "quad":
            state_mgr.set_motors( [ actpt["output[0]"],
                                          actpt["output[1]"],
                                          actpt["output[2]"],
                                          actpt["output[3]"] ] )
    if "gps" in record:
        gpspt = record["gps"]
    if "air" in record:
        airpt = record["air"]

        asi_mps = airpt["airspeed"] * kt2mps
        # add in correction factor if available
        if pitot_scale is not None:
            asi_mps *= pitot_scale
        elif "pitot_scale" in airpt:
            asi_mps *= airpt["pitot_scale"]
        if "alpha" in airpt and "beta" in airpt:
            state_mgr.set_airdata( asi_mps, airpt["alpha"]*d2r, airpt["beta"]*d2r )
        else:
            state_mgr.set_airdata( asi_mps )
        if wn_interp is not None and we_interp is not None:
            # post process wind estimate
            wn = wn_interp(imupt["time"])
            we = we_interp(imupt["time"])
            wd = 0
        elif "wind_dir" in airpt:
            wind_psi = 0.5 * pi - airpt["wind_dir"] * d2r
            wind_mps = airpt["wind_speed"] * kt2mps
            we = cos(wind_psi) * wind_mps
            wn = sin(wind_psi) * wind_mps
            wd = 0
        elif False and flight_format == "cirrus_csv" and state_mgr.is_flying():
            windest.update(imupt["time"], airpt["airspeed"], navpt["psi"], navpt["vn"], navpt["ve"])
            wn = windest.filt_long_wn.value
            we = windest.filt_long_we.value
            wd = 0
            print("%.2f %.2f" % (wn, we))
    if "filter" in record:
        navpt = record["filter"]
        psi = navpt["psi"]
        if psi_bias is not None:
            psi += psi_bias
        state_mgr.set_orientation( navpt["phi"], navpt["the"], navpt["psi"] )
        state_mgr.set_pos(navpt["lon"], navpt["lat"], navpt["alt"])
        if args.vehicle == "wing" or np.linalg.norm([navpt["vn"], navpt["ve"], navpt["vd"]]) > 0.000001:
            state_mgr.set_ned_velocity( navpt["vn"], navpt["ve"],
                                              navpt["vd"], wn, we, wd )
        else:
            state_mgr.set_ned_velocity( gpspt["vn"], gpspt["ve"],
                                              gpspt["vd"], wn, we, wd )

    # Our model is only valid during flight aloft, skip non-flying data points
    if not state_mgr.is_flying():
        continue

    # 2. Derived states
    state_mgr.compute_derived_states(state_mgr.have_alpha)

    # 3. Compute terms (combinations of states and derived states)
    state_mgr.compute_terms()

    state = state_mgr.gen_state_vector(train_states)
    # print(state_mgr.state2dict(state))
    for i, condition in enumerate(conditions):
        # print(i, condition)
        if "flaps" in condition and abs(state_mgr.flaps - condition["flaps"]) < 0.1:
            # print(True)
            cond_list[i]["traindata_list"].append( state )
            if False and args.vehicle == "wing":
                params = [ state_mgr.alpha*r2d, state_mgr.Cl_raw, 0, state_mgr.qbar,
                            state_mgr.accels[0], state_mgr.throttle ]
                # print("params:", params)
                cond_list[i]["coeff"].append( params )

print("Conditions report:")
for i, cond in enumerate(conditions):
    print(i, cond)
    print("  Number of states:", len(cond_list[i]["traindata_list"][0]))
    print("  Input state vectors:", len(cond_list[i]["traindata_list"]))

# stub / top of our model structure to save
root_dict = {
    "dt": imu_dt,
    "rows": len(state_mgr.output_states),
    "cols": len(state_mgr.state_list),
    "conditions": [],
}

def correlation_report_3(train_states, traindata, fit_states):
    print("test pearson correlation coefficients:")
    print(train_states)
    corr = np.corrcoef(traindata.T)
    print(corr)

    # pick the next most correlating state that correlates the least with already chosen states
    for s in fit_states:
        i = train_states.index(s)
        print(train_states[i] + ": ", end="")
        row = corr[i,:]
        incl = {}
        rem = {}
        for j in range(len(row)):
            if i != j:
                rem[j] = 0
        # print()
        while len(rem):
            # score remaining states based on correlation with state[i] minus max(correlation with allocated states)
            for j in rem.keys():
                # print(" ", state_list[j])
                max_corr = 0
                for k in incl.keys():
                    c = abs(corr[j,k])
                    if c > max_corr:
                        max_corr = c
                rem[j] = abs(corr[i,j]) - max_corr
            idx = sorted(rem.items(), key=lambda item: item[1], reverse=True)
            # print(idx)
            # print("choose:", idx[0][0], state_list[idx[0][0]])
            print("%.3f" % row[idx[0][0]], train_states[idx[0][0]] + ", ", end="")
            del rem[idx[0][0]]
            incl[idx[0][0]] = True
        print("")


# evaluate each condition
for i, cond in enumerate(conditions):
    print(i, cond)
    condition_dict = { "condition": cond }
    traindata = np.array(cond_list[i]["traindata_list"])
    coeff = np.array(cond_list[i]["coeff"])

    sysid = SystemIdentification(args.vehicle)
    cond_list[i]["sysid"] = sysid

    correlation_report_3(train_states, traindata, output_states)
