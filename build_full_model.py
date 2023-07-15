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

from flightdata import flight_loader, flight_interp

from lib.constants import d2r, r2d, kt2mps
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

sysid = SystemIdentification(args.vehicle)

if flight_format == "cirrus_csv":
    independent_states = [
        "aileron",                  # flight controls (* qbar)
        "elevator",
        "rudder",
        "flaps",
        "thrust",                   # based on throttle
        "drag",                     # (based on flow accel, and flow g
        "bgx", "bgy", "bgz",        # gravity rotated into body frame
        # "abs(bgy)",               # gravity effects related to abs(bank angle)
        "sin(alpha)",               # angle of attack
        "sin(beta)",                # side slip
        # "bvx", "bvy", "bvz",      # (direct alpha/beta better) velocity components (body frame)
    ]
    dependent_states = [
        "airspeed",
        "bax", "bay", "baz",        # acceleration in body frame (no g)
        "p", "q", "r",              # imu (body) rates
    ]
elif args.vehicle == "wing":
    independent_states = [
        "aileron", "abs(aileron)",
        "elevator",
        "rudder", "abs(rudder)",    # flight controls (* qbar)
        #"flaps",
        "thrust",                   # based on throttle
        "drag",                     # (based on flow accel, and flow g
        "bgx", "bgy", "bgz",        # gravity rotated into body frame
        "bvx", "bvy", "bvz",        # velocity components (body frame)
    ]
    dependent_states = [
        "airspeed",
        "bax", "bay", "baz",        # acceleration in body frame (no g)
        "p", "q", "r",              # imu (body) rates
    ]
elif args.vehicle == "quad":
    independent_states = [
        "motor[0]",
        "motor[1]",
        "motor[2]",
        "motor[3]",                  # motor commands
        #"bgx", "bgy", "bgz",        # gravity rotated into body frame
        #"ax", "ay", "az",           # imu (body) accels
        #"bax", "bay", "baz",        # acceleration in body frame (no g)
    ]
    dependent_states = [
        #"bvx", "bvy",
        "bvz",                        # velocity components (body frame)
        #"p", "q", "r",               # imu (body) rates
    ]

state_names = independent_states + dependent_states
sysid.state_mgr.set_state_names(independent_states, dependent_states)

if flight_format == "cirrus_csv":
    sysid.state_mgr.set_is_flying_thresholds(60*kt2mps, 50*kt2mps)

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

sysid.state_mgr.set_dt(imu_dt)

print("Parsing flight data log:")
actpt = {}
airpt = {}
navpt = {}
g = np.array( [ 0, 0, -9.81 ] )

wn = 0
we = 0
wd = 0

coeff = []

pitot_scale = None
psi_bias = None
wn_interp = None
we_interp = None

# backup wind estimator if needed
windest = Wind()

if True or flight_format != "cirrus_csv":
    # note: wind estimates are only needed for estimating alpha/beta (or
    # bvz/bvy) which is not needed if the aircraft is instrumented with
    # alpha/beta vanes and these are measured directly.

    from lib.wind2 import Wind2
    w2 = Wind2()
    pitot_scale, psi_bias, wn_interp, we_interp = w2.estimate( flight_interp.IterateGroup(data), imu_dt )

# iterate through the flight data log, cherry pick the selected parameters
iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    record = iter.next()
    if len(record) == 0:
        continue
    if "filter" in record:
        # need ahead of air in case we are doing a wind estimate
        navpt = record["filter"]
    else:
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
        ax = imupt["ax"]
        ay = imupt["ay"]
        az = imupt["az"]
        if "ax_bias" in navpt:
            ax -= navpt["ax_bias"]
            ay -= navpt["ay_bias"]
            az -= navpt["az_bias"]
        sysid.state_mgr.set_accels(ax, ay, az)
    if "act" in record:
        actpt = record["act"]
        if args.vehicle == "wing":
            sysid.state_mgr.set_throttle( actpt["throttle"] )
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
            sysid.state_mgr.set_flight_surfaces( ail, ele, rud, flaps )
        elif args.vehicle == "quad":
            sysid.state_mgr.set_motors( [ actpt["output[0]"],
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
            sysid.state_mgr.set_airdata( asi_mps, airpt["alpha"]*d2r, airpt["beta"]*d2r )
        else:
            sysid.state_mgr.set_airdata( asi_mps )
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
        elif flight_format == "cirrus_csv" and sysid.state_mgr.is_flying():
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
        sysid.state_mgr.set_orientation( navpt["phi"], navpt["the"], navpt["psi"] )
        sysid.state_mgr.set_pos(navpt["lon"], navpt["lat"], navpt["alt"])
        if args.vehicle == "wing" or np.linalg.norm([navpt["vn"], navpt["ve"], navpt["vd"]]) > 0.000001:
            sysid.state_mgr.set_ned_velocity( navpt["vn"], navpt["ve"],
                                              navpt["vd"], wn, we, wd )
        else:
            sysid.state_mgr.set_ned_velocity( gpspt["vn"], gpspt["ve"],
                                              gpspt["vd"], wn, we, wd )

    if sysid.state_mgr.is_flying():
        if flight_format == "cirrus_csv":
            sysid.state_mgr.compute_body_frame_values(compute_body_vel=True, have_alpha_beta=True)
        else:
            sysid.state_mgr.compute_body_frame_values(compute_body_vel=True)
        state = sysid.state_mgr.gen_state_vector()
        #print(sysid.state_mgr.state2dict(state))
        sysid.add_state_vec(state)
        if args.vehicle == "wing":
            coeff.append( [sysid.state_mgr.alpha*r2d,
                           sysid.state_mgr.Cl, sysid.state_mgr.Cd,
                           sysid.state_mgr.airspeed_mps,
                           sysid.state_mgr.drag] )

if True and len(coeff):
    coeff = np.array(coeff)

    # alpha vs Cl, Cd plot
    num_bins = 25
    bins = np.linspace(-5, 20, num_bins+1) # alpha range
    print(bins)

    bin_indices = np.digitize( coeff[:,0], bins )

    d1 = []
    for i in range(num_bins):
        bin = i + 1
        cl_mean = np.mean(coeff[bin_indices==i+1,1])
        cd_mean = np.mean(coeff[bin_indices==i+1,2])
        if not np.isnan(cl_mean):
            pt = 0.5 * (bins[i] + bins[i+1])
            print( i, pt, cl_mean, cd_mean )
            d1.append( [pt, cl_mean, cd_mean] )
    d1 = np.array(d1)
    plt.figure()
    plt.plot(d1[:,0], d1[:,1], label="Cl vs. alpha (deg)")
    plt.xlabel("alpha (deg)")
    plt.legend()

    # asi vs drag
    num_bins = 25
    max = np.max(coeff[:,3])
    bins = np.linspace(0, max, num_bins+1) # alpha range
    print(bins)

    bin_indices = np.digitize( coeff[:,3], bins )

    d1 = []
    for i in range(num_bins):
        bin = i + 1
        drag_mean = np.mean(coeff[bin_indices==i+1,4])
        if not np.isnan(drag_mean):
            pt = 0.5 * (bins[i] + bins[i+1])
            print( i, pt, drag_mean )
            d1.append( [pt, drag_mean] )
    d1 = np.array(d1)
    plt.figure()
    plt.plot(d1[:,0], d1[:,1], label="airspeed vs drag")
    plt.xlabel("airspeed (mps)")
    plt.legend()
    plt.show()

print("Number of states:", len(sysid.traindata_list[0]))
print("Input state vectors:", len(sysid.traindata_list))

sysid.fit()
sysid.model_noise()
sysid.analyze()
sysid.save(args.write, imu_dt)

# show a running estimate of dependent states.  Feed the dependent estimate
# forward into next state rather than using the original logged value.  This can
# show the convergence of the estimated parameters versus truth (or show major
# problems in the model.)

# the difference here is we are propagating the prediction forward as if we
# don't have other knowledge of the dependent states (i.e. what would happen in
# a flight simulation, or if we used this system to emulate airspeed or imu
# sensors?)

dep_index_list = sysid.state_mgr.get_state_index( dependent_states )
#print("dep_index list:", dep_index_list)
est_val = [0.0] * len(dependent_states)
pred = []
v = []
for i in range(len(sysid.traindata)):
    v  = sysid.traindata[i].copy()
    for j, index in enumerate(dep_index_list):
        v[index] = est_val[j]
    #print("A:", A.shape, A)
    #print("v:", np.array(v).shape, np.array(v))
    p = sysid.A @ np.array(v)
    #print("p:", p)
    for j, index in enumerate(dep_index_list):
        est_val[j] = p[j]
        param = sysid.model["parameters"][index]
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

index_list = sysid.state_mgr.get_state_index( dependent_states )
for j in range(len(index_list)):
    plt.figure()
    plt.plot(np.array(sysid.traindata).T[index_list[j],:], label="%s (orig)" % state_names[index_list[j]])
    plt.plot(Ypred[j,:], label="%s (pred)" % state_names[index_list[j]])
    plt.legend()
plt.show()

