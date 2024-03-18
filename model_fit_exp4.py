#!/usr/bin/env python3

"""build_full_model

Attempt to use a DMD-esque approach to fit a state transition matrix
that maps previous state to next state, thereby modeling/simulating
flight that closely approximates the original real aircraft.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
import dask.array as da         # dnf install python3-dask+array
from matplotlib import pyplot as plt
import numpy as np

from lib.constants import kt2mps
from lib.state_mgr import StateManager
from lib.system_id import SystemIdentification
from lib.traindata import TrainData

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
        "sin(alpha_deg)*qbar",
        "sin(beta_deg)*qbar",
        "abs(ay)", "abs(bgy)",
        # state history can improve fit and output parameter prediction, but reduce determinism
        "sin(alpha_prev1_deg)*qbar", "sin(beta_prev1_deg)*qbar",
        "ax_prev1", "ay_prev1", "az_prev1",
        "p_prev1", "q_prev1", "r_prev1",
    ]

    # states to predict
    output_states = [
        "p", "q", "r",
        "ax", "ay", "az",
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

def solve(traindata, includes_idx, solutions_idx):
    srcdata = traindata[includes_idx,:]
    soldata = traindata[solutions_idx,:]
    states = len(traindata[0])

    X = np.array(srcdata[:,:-1])
    Y = np.array(soldata[:,1:])
    print("X:", X.shape)
    print("Y:", Y.shape)
    # print("X:\n", np.array(X))
    # print("Y:\n", np.array(Y))

    # Y = A * X, solve for A
    #
    # A is a matrix that projects (predicts) all the next states
    # given all the previous states (in a least squares best fit
    # sense)
    #
    # X isn't nxn and doesn't have a direct inverse, so first
    # perform an svd:
    #
    # Y = A * U * D * V.T

    # print("dask svd...")
    daX = da.from_array(X, chunks=(X.shape[0], 10000)).persist()
    u, s, vh = da.linalg.svd(daX)

    if False:
        # debug and sanity check
        print("u:\n", u.shape, u)
        print("s:\n", s.shape, s)
        print("vh:\n", vh.shape, vh)
        Xr = (u * s) @ vh[:states, :]
        print( "dask svd close?", np.allclose(X, Xr.compute()) )

    # after algebraic manipulation
    #
    # A = Y * V * D.inv() * U.T

    v = vh.T
    # print("s inv:", (1/s).compute() )

    A = (Y @ (v[:,:states] * (1/s)) @ u.T).compute()
    print("A rank:", np.linalg.matrix_rank(A))
    print("A:\n", A.shape, A)

    return A

def simulate(traindata, includes_idx, solutions_idx, A):
    est = []
    next = np.zeros(len(solutions_idx))
    est.append(next)
    for i in range(traindata.shape[1]-1):
        # print("i:", i)
        # print("includes_idx:", includes_idx)
        # print("solutions_idx:", solutions_idx)
        v = traindata[includes_idx,i]
        # print(v.shape, v)
        v[solutions_idx] = next
        next = A @ v
        est.append(next)
    return np.array(est).T

def rms(y):
    return np.sqrt(np.mean(y**2))

def correlation_report_4(traindata, train_states, output_states, self_reference=False):
    outputs_idx = []
    for s in output_states:
        outputs_idx.append(train_states.index(s))

    inputs_idx = []
    for i in range(len(train_states)):
        if not self_reference:
            if i in outputs_idx:
                continue
        inputs_idx.append(i)

    A = solve(traindata, inputs_idx, outputs_idx)

    if False:
        est = A @ traindata[inputs_idx,:]
        error = traindata[outputs_idx,1:] - est[:,:-1]
        for i in range(error.shape[0]):
            print("ERROR:", output_states[i], rms(error[i,:]), "%.3f%%" % (100 * rms(error[i,:]) / rms(est[i,:]) ))
            plt.figure()
            plt.plot(error[i,:].T, label="estimation error")
            plt.plot(traindata[outputs_idx[i],1:].T, label="original signal")
            plt.plot(est[i,:-1].T, label="estimated signal")
            plt.legend()
        plt.show()

    est = simulate(traindata,inputs_idx, outputs_idx, A)
    print("est:", est.shape)
    error = traindata[outputs_idx,1:] - est[:,:-1]
    for i in range(error.shape[0]):
        print("ERROR:", output_states[i], rms(error[i,:]), "%.3f%%" % (100 * rms(error[i,:]) / rms(est[i,:]) ))
        plt.figure()
        plt.plot(error[i,:].T, label="estimation error")
        plt.plot(traindata[outputs_idx[i],1:].T, label="original signal")
        plt.plot(est[i,:-1].T, label="estimated signal")
        plt.legend()
    plt.show()


# evaluate each condition
for i, cond in enumerate(conditions):
    print(i, cond)
    condition_dict = { "condition": cond }
    traindata = np.array(train_data.cond_list[i]["traindata_list"]).T
    coeff = np.array(train_data.cond_list[i]["coeff"])

    sysid = SystemIdentification(args.vehicle)
    train_data.cond_list[i]["sysid"] = sysid

    correlation_report_4(traindata, train_states, output_states, self_reference=True)
