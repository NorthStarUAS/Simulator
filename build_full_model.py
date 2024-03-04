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

if False and flight_format == "cirrus_csv":
    # experiment with an alpha and alpha_dot estimator
    inceptor_states = [
        # flight controls (* qbar)
        "aileron",
        "elevator",
        "rudder",
        "throttle",
        "ax",                       # thrust - drag
        "ay",                       # side force
        "az",                       # lift
        # "p", "q", "r",              # imu (body) rates
        "q",              # pitch rate
    ]
    internal_states = [
        "abs(aileron)", "abs(elevator)", "abs(rudder)",
        "bgx", "bgy", "bgz",        # gravity rotated into body frame
        # "qbar",                     # effects due to dynamic pressure
        # "inv_airspeed_mps",             # effects due to airspeed
        # additional state history improves fit and output parameter prediction.
        # "alpha_prev1", "beta_prev1",
        # "alpha",
        # "alpha_prev1",
        "alpha_dot_term2",
        "alpha_dot_term3",
        # "ax_prev1", "ay_prev1", "az_prev1",
        # "p_prev1", "q_prev1", "r_prev1",
        "q_prev1",
        "abs(ay)", "abs(bgy)",
        "K",                        # constant factor (1*parameter)
    ]
    output_states = [
        # "alpha", "beta",            # angle of attack, side slip angle
        "alpha",
        # "alpha_dot",
    ]
    conditions = [
        { "flaps": 0 },
        { "flaps": 0.5 }
    ]
elif False and flight_format == "cirrus_csv":
    # experiment with a minimal beta estimator
    inceptor_states = [
        # flight controls (* qbar)
        # "aileron",
        # "elevator",
        "rudder",
        # "throttle",
        # "ax",                       # thrust - drag
        "ay",                       # side force
        # "az",                       # lift
        # "p", "q", "r",              # imu (body) rates
        "r",
    ]
    internal_states = [
        # "abs(aileron)",
        # "abs(rudder)",
        "bgy",
        # "bgx", "bgy", "bgz",        # gravity rotated into body frame
        "qbar",                     # effects due to dynamic pressure
        # additional state history improves fit and output parameter prediction.
        # "alpha_prev1", "beta_prev1",
        "beta_prev1",
        # "ax_prev1", "ay_prev1", "az_prev1",
        "ay_prev1",
        # "p_prev1", "q_prev1", "r_prev1",
        "r_prev1",
        # "abs(ay)", "abs(bgy)",
        # "K",                        # constant factor (1*parameter)
    ]
    output_states = [
        # "alpha", "beta",            # angle of attack, side slip angle
        "beta",
    ]
    conditions = [
        { "flaps": 0 },
        { "flaps": 0.5 }
    ]
elif False and flight_format == "cirrus_csv":
    # question: seem to get a better flaps up fit to airspeed (vs. qbar) but fails to converge for 50% flaps
    # qbar only converges for both conditions
    inceptor_states = [
        # flight controls (* qbar)
        # "aileron",
        # "elevator",
        "rudder",
        "throttle",
        "ax",                       # thrust - drag
        "ay",                       # side force
        "az",                       # lift
        "p", "q", "r",              # imu (body) rates
]
    internal_states = [
        # "abs(aileron)",
        # "abs(rudder)",
        "bgx", "bgy", "bgz",        # gravity rotated into body frame
        # additional state history improves fit and output parameter prediction.
        # "alpha_prev1", "beta_prev1",
        "ax_prev1", "ay_prev1", "az_prev1",
        "p_prev1", "q_prev1", "r_prev1",
        "abs(ay)", "abs(bgy)",
        "qbar",                     # effects due to airspeed airframe
        # "K",                        # constant factor (1*parameter)
    ]
    output_states = [
        "airspeed_mps",
        # "alpha", "beta",            # angle of attack, side slip angle
        "alpha",
    ]
    conditions = [
        { "flaps": 0 },
        { "flaps": 0.5 }
    ]
elif flight_format == "cirrus_csv":
    # question 1: seem to get a better flaps up fit to airspeed (vs. qbar) but fails to converge for 50% flaps
    # qbar only converges for both conditions
    # question 2: would it be useful to have a gamma (flight path angle) parameter (may help asi)
    inceptor_states = [
        # flight controls (* qbar)
        "aileron",
        "elevator",
        "rudder",
        "throttle",
    ]
    internal_states = [
        "abs(aileron)",
        "abs(rudder)",
        "bgx", "bgy", "bgz",        # gravity rotated into body frame
        # additional state history improves fit and output parameter prediction.
        "alpha_dot_term2",
        "alpha_dot_term3",
        "alpha_prev1", "beta_prev1",
        "ax_prev1", "ay_prev1", "az_prev1",
        "p_prev1", "q_prev1", "r_prev1",
        "abs(ay)", "abs(bgy)",
        "qbar",                     # effects due to airspeed airframe
        "K",                        # constant factor (1*parameter)
    ]
    output_states = [
        "airspeed_mps",
        "alpha", "beta",            # angle of attack, side slip angle
        "ax",                       # thrust - drag
        "ay",                       # side force
        "az",                       # lift
        "p", "q", "r",              # imu (body) rates
    ]
    conditions = [
        { "flaps": 0 },
        { "flaps": 0.5 }
    ]
elif args.vehicle == "wing":
    internal_states = [
        "aileron", "abs(aileron)",
        "elevator",
        "rudder", "abs(rudder)",    # flight controls (* qbar)
        #"flaps",
        "thrust",                   # based on throttle
        "drag",                     # (based on flow accel, and flow g
        "bgx", "bgy", "bgz",        # gravity rotated into body frame
        "bvx", "bvy", "bvz",        # velocity components (body frame)
    ]
    output_states = [
        "airspeed",
        "bax", "bay", "baz",        # acceleration in body frame (no g)
        "p", "q", "r",              # imu (body) rates
    ]
elif args.vehicle == "quad":
    internal_states = [
        "motor[0]",
        "motor[1]",
        "motor[2]",
        "motor[3]",                  # motor commands
        #"bgx", "bgy", "bgz",        # gravity rotated into body frame
        #"ax", "ay", "az",           # imu (body) accels
        #"bax", "bay", "baz",        # acceleration in body frame (no g)
    ]
    output_states = [
        #"bvx", "bvy",
        "bvz",                        # velocity components (body frame)
        #"p", "q", "r",               # imu (body) rates
    ]

state_names = inceptor_states + internal_states + output_states
state_mgr.set_state_names(inceptor_states, internal_states, output_states)

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

# iterate through the flight data log (a sequence of time samples of all the measured states)
iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
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

    state = state_mgr.gen_state_vector()
    # print(state_mgr.state2dict(state))
    for i, condition in enumerate(conditions):
        # print(i, condition)
        if "flaps" in condition and abs(state_mgr.flaps - condition["flaps"]) < 0.1:
            # print(True)
            cond_list[i]["traindata_list"].append( state )
            if args.vehicle == "wing":
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

# create a solution for each condition
for i, cond in enumerate(conditions):
    print(i, cond)
    condition_dict = { "condition": cond }
    traindata = np.array(cond_list[i]["traindata_list"])
    coeff = np.array(cond_list[i]["coeff"])

    sysid = SystemIdentification(args.vehicle)
    cond_list[i]["sysid"] = sysid

    sysid.compute_lift_curve(coeff)
    sysid.fit(state_mgr, traindata)
    sysid.model_noise(state_mgr, traindata)
    sysid.analyze(state_mgr, traindata)
    condition_dict["parameters"] = sysid.parameters
    condition_dict["A"] = sysid.A.flatten().tolist()
    root_dict["conditions"].append(condition_dict)

# the median delta t from the data log is important to include
# with the state transition matrix because the state
# transition matrix coefficients assume this value for
# realtime performance.

f = open(args.write, "w")
json.dump(root_dict, f, indent=4)
f.close()

if True:
    # for each condition, show a running estimate of output states.  Feed the
    # output estimate forward into next state rather than using the original
    # logged value.  This can show the convergence of the estimated parameters
    # versus truth (or show major problems in the model fit.)

    # the difference here is we are propagating the prediction forward as if we
    # don't have any external knowledge of the output states (i.e. this is what
    # would happen in a flight simulation, or if we used this system to emulate
    # airspeed or imu sensors?)
    for i, cond in enumerate(conditions):
        sysid = cond_list[i]["sysid"]
        traindata = np.array(cond_list[i]["traindata_list"])
        output_index_list = state_mgr.get_state_index( output_states )
        #print("output_index list:", output_index_list)
        est_val = [0.0] * len(output_states)
        pred = []
        v = []
        for i in range(len(traindata)):
            v  = traindata[i].copy()
            for j, index in enumerate(output_index_list):
                v[index] = est_val[j]
            #print("A:", A.shape, A)
            #print("v:", np.array(v).shape, np.array(v))
            p = sysid.A @ np.array(v)
            #print("p:", p)
            for j, index in enumerate(output_index_list):
                est_val[j] = p[j]
                param = sysid.parameters[index]
                min = param["min"]
                max = param["max"]
                med = param["median"]
                std = param["std"]
                #if est_val[j] < med - 2*std: est_val[j] = med - 2*std
                #if est_val[j] > med + 2*std: est_val[j] = med + 2*std
                if est_val[j] < min - std: est_val[j] = min - std
                if est_val[j] > max + std: est_val[j] = max + std
            pred.append(p)
        Ypred = np.array(pred).T

        index_list = state_mgr.get_state_index( output_states )
        for j in range(len(index_list)):
            plt.figure()
            plt.plot(np.array(traindata).T[index_list[j],:], label="%s (orig)" % state_names[index_list[j]])
            plt.plot(Ypred[j,:], label="%s (pred)" % state_names[index_list[j]])
            plt.legend()
        plt.show()

if False:
    # a more adventurous test!

    # show a running estimate of output states.  Feed the output estimate
    # forward into next state rather than using the original logged value.  This can
    # show the convergence of the estimated parameters versus truth (or show major
    # problems in the model.)

    # the difference here is we are propagating the prediction forward as if we
    # don't have other knowledge of the output states (i.e. what would happen in
    # a flight simulation, or if we used this system to emulate airspeed or imu
    # sensors?)

    from lib.simulator import Simulator
    from lib.state_mgr import StateManager
    sim = Simulator()
    sim.load(args.write)
    sim.reset()

    incepter_list = ["aileron", "elevator", "rudder", "throttle",]
    inc_index_list = state_mgr.get_state_index( incepter_list )

    simdata = [ traindata[0] ]
    for i in range(len(traindata)):
        v  = traindata[i].copy()
        # set incepters from original data
        sim.state_mgr.set_throttle(v[inc_index_list[3]])
        sim.state_mgr.set_flight_surfaces(v[inc_index_list[0]], v[inc_index_list[1]], v[inc_index_list[2]] )
        sim.update()
        state = sim.state_mgr.gen_state_vector()
        simdata.append(state)
    simdata = np.array(simdata)

    index_list = state_mgr.get_state_index( state_names )
    for i in range(len(index_list)):
        plt.figure()
        plt.plot(np.array(traindata).T[index_list[i],:], label="%s (orig)" % state_names[index_list[i]])
        plt.plot(np.array(simdata).T[index_list[i],:], label="%s (sim)" % state_names[index_list[i]])
        plt.legend()
    plt.show()

