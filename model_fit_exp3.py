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
from lib.traindata import TrainData
from lib.wind import Wind

# command line arguments
parser = argparse.ArgumentParser(description="build full model")
parser.add_argument("flight", help="flight data log")
parser.add_argument("--write", required=True, help="write model file name")
parser.add_argument("--vehicle", default="wing", choices=["wing", "quad"], help="vehicle type represented by data file")
parser.add_argument("--invert-elevator", action='store_true', help="invert direction of elevator")
parser.add_argument("--invert-rudder", action='store_true', help="invert direction of rudder")
args = parser.parse_args()

state_mgr = StateManager(args.vehicle)

train_data = TrainData()
train_data.load_flightdata(args.flight)

if train_data.flight_format == "cirrus_csv":
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

if train_data.flight_format == "cirrus_csv":
    state_mgr.set_is_flying_thresholds(75*kt2mps, 65*kt2mps)

train_data.build(args.vehicle, args.invert_elevator, args.invert_rudder, state_mgr, conditions, train_states)

print("Conditions report:")
for i, cond in enumerate(conditions):
    print(i, cond)
    print("  Number of states:", len(train_data.cond_list[i]["traindata_list"][0]))
    print("  Input state vectors:", len(train_data.cond_list[i]["traindata_list"]))

# stub / top of our model structure to save
root_dict = {
    "dt": state_mgr.dt,
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
    traindata = np.array(train_data.cond_list[i]["traindata_list"])
    coeff = np.array(train_data.cond_list[i]["coeff"])

    sysid = SystemIdentification(args.vehicle)
    train_data.cond_list[i]["sysid"] = sysid

    correlation_report_3(train_states, traindata, output_states)
