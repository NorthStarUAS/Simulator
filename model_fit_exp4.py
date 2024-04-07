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
    inceptor_terms = [
        "aileron",
        "elevator",
        "rudder",
        "throttle",
    ]

    # sensors (directly sensed, or directly converted)
    inertial_terms = [
        "p", "q", "r",        # imu (body) rates
        "ax",                 # thrust - drag
        "ay",                 # side force
        "az",                 # lift
        "bgx", "bgy", "bgz",  # gravity rotated into body frame
        "abs(ay)", "abs(bgy)",
        # "ax_1", "ax_2", "ax_3", "ax_4",
        # "ay_1", "ay_2", "ay_3", "ay_4",
        # "az_1", "az_2", "az_3", "az_4",
        # "p_1", "p_2", "p_3", "p_4",
        # "q_1", "q_2", "q_3", "q_4",
        # "r_1", "r_2", "r_3", "r_4",
    ]

    airdata_terms = [
        "airspeed_mps",
        # "alpha_dot",
        "alpha_deg",          # angle of attack
        "beta_deg",           # side slip angle
        "qbar",
        "1/qbar",
        "1/airspeed_mps",
        # "alpha_dot_term2",
        # "sin(alpha_deg)*qbar", "sin(alpha_deg)*qbar_1",
        # "sin(beta_deg)*qbar", "sin(beta_deg)*qbar_1",
        # "qbar/cos(beta_deg)",
    ]

    inceptor_airdata_terms = [
        # "aileron*qbar", "aileron*qbar_1",
        # "abs(aileron)*qbar",
        # "elevator*qbar", "elevator*qbar_1", "elevator*qbar_2", "elevator*qbar_3",
        # "rudder*qbar", "rudder*qbar_1", "rudder*qbar_2", "rudder*qbar_3",
        # "abs(rudder)*qbar",
    ]

    inertial_airdata_terms = [
        "Cl",
        # "alpha_dot_term3",
    ]

    # deterministic output states (do not include their own value in future estimates)
    output_states = [
        "aileron",
        "q",
        "beta_deg",
        "q",
        # "alpha_deg",
        "beta_deg",
    ]

    # non-deterministic output states (may roll their current value into the next estimate)
    output_states_2 = [
        "airspeed_mps",
        "p", "q", "r",
        "ax", "ay", "az",
    ]

    # bins of unique flight conditions
    conditions = [
        { "flaps": 0 },
        { "flaps": 0.5 }
    ]

train_states = inceptor_terms + inceptor_airdata_terms + inertial_terms + airdata_terms
# train_states = inceptor_terms + inertial_terms + airdata_terms
state_mgr.set_state_names(inceptor_terms, inertial_terms + airdata_terms, output_states)

# previous state propagation
propagate = []
for i, s in enumerate(train_states):
    if len(s) >= 3 and s[-2] == "_":
        print("evaluating:", s)
        n = int(s[-1])
        root = s[:-2]
        if n == 1:
            if root in train_states:
                src = train_states.index(root)
            else:
                print("ERROR: requested state history without finding the current state:", s, "->", root)
                quit()
        else:
            newer = root + "_%d" % (n-1)
            if newer in train_states:
                src = train_states.index(newer)
            else:
                print("ERROR: requested state history without finding the current state:", s, "->", newer)
                quit()
        dst = i
        propagate.append( [src, dst] )
print("Previous state propagation:", propagate)

if train_data.flight_format == "cirrus_csv":
    state_mgr.set_is_flying_thresholds(75*kt2mps, 65*kt2mps)

train_data.build(args.vehicle, args.invert_elevator, args.invert_rudder, state_mgr, conditions, train_states)

print("Conditions report:")
for i, cond in enumerate(conditions):
    print(i, cond)
    print("  Number of states:", len(train_data.cond_list[i]["traindata_list"][0]))
    print("  Input state vectors:", len(train_data.cond_list[i]["traindata_list"]))

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

def analyze(A, traindata, train_states, output_states):
    stds = []
    for i in range(len(train_states)):
        stds.append(np.std(traindata[i,:]))

    # output_index_list = state_mgr.get_state_index( state_mgr.output_states )
    # states = len(traindata[0])
    # params = self.parameters

    # report leading contributions towards computing each output state
    for i in range(len(output_states)):
        #print(self.state_names[i])
        row = A[i,:]
        energy = []
        for j in range(len(train_states)):
            # e = row[j] * (abs(params[j]["median"]) + 0.5 * params[j]["std"]) * np.sign(params[j]["median"])
            e = row[j] * stds[j]
            # e = row[j] * params[j]["median"]
            # e = row[j] * (params[j]["max"] - params[j]["min"]) # probably no ...
            energy.append(e)
        idx = np.argsort(-np.abs(energy))
        total = np.sum(np.abs(energy))
        # output_idx = output_index_list[i]
        contributors = output_states[i] + " = "
        formula = output_states[i] + " = "
        first = True
        for j in idx:
            perc = 100 * energy[j] / total
            if abs(perc) < 0.01:
                continue
            if first:
                first = False
            else:
                if perc >= 0:
                    contributors += " + "
                else:
                    contributors += " - "
            if row[j] < 0:
                formula += " - "
            else:
                formula += " + "
            contributors += train_states[j] + " %.1f%%" % abs(perc)
            formula += "%.3f" % abs(row[j]) + "*" + train_states[j]
        print(contributors)
        print(formula)

def simulate(traindata, includes_idx, solutions_idx, A):
    # make a copy because we are going to roll our state estimates through the
    # data matrix and make a mess (or a piece of artwork!) out of it.
    data = traindata.copy()

    # this gets a little funky because we will be using numpy implied indexing below.
    indirect_idx = []
    for i in solutions_idx:
        if i in includes_idx:
            indirect_idx.append( includes_idx.index(i) )

    if False: # we don't need this
        # more craziness ... the propagate (state history) mapping is relative to
        # the full traindata so we need to indirectly index those as well
        local_prop = []
        for [src, dst] in propagate:
            if src in includes_idx and dst in includes_idx:
                local_prop.append( [includes_idx.index(src), includes_idx.index(dst)] )

    def shuffle_down(j):
        if j < data.shape[1] - 1:
            for [src, dst] in reversed(propagate):
                data[dst,j+1] = data[src,j]

    est = []
    next = np.zeros(len(indirect_idx))
    data[solutions_idx,i] = next
    for i in range(data.shape[1]):
        # print("i:", i)
        # print("includes_idx:", includes_idx)
        # print("solutions_idx:", solutions_idx)
        v = data[includes_idx,i]
        # print(v.shape, v)
        if len(indirect_idx):
            v[indirect_idx] = next
        next = A @ v
        shuffle_down(i)
        if i < data.shape[1] - 1:
            data[solutions_idx,i+1] = next
        est.append(next)
    return np.array(est).T

def rms(y):
    # return np.sqrt(np.mean(y**2))
    return np.std(y)

def mass_solution_4(traindata, train_states, output_states, self_reference=False):
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

    # direct solution with all current states known, how well does our fit estimate the next state?
    direct_est = A @ traindata[inputs_idx,:]
    direct_error = traindata[outputs_idx,1:] - direct_est[:,:-1]

    sim_est = simulate(traindata,inputs_idx, outputs_idx, A)
    sim_error = traindata[outputs_idx,1:] - sim_est[:,:-1]

    analyze(A, traindata, train_states, output_states)

    for i in range(len(output_states)):
        print("rms vs std:", rms(direct_error[i,:]), np.std(direct_error[i,:]))
        print("ERROR Direct:", output_states[i], rms(direct_error[i,:]), "%.3f%%" % (100 * rms(direct_error[i,:]) / rms(direct_est[i,:]) ))
        print("ERROR Sim:", output_states[i], rms(sim_error[i,:]), "%.3f%%" % (100 * rms(sim_error[i,:]) / rms(sim_est[i,:]) ))

        fig, axs = plt.subplots(2, sharex=True)
        fig.suptitle("Estimate for: " + output_states[i])
        axs[0].plot(traindata[outputs_idx[i],1:].T, label="original signal")
        axs[0].plot(direct_est[i,:-1].T, label="fit signal")
        axs[0].plot(sim_est[i,:-1].T, label="sim signal")
        axs[0].legend()
        axs[1].plot(direct_error[i,:].T, label="fit error")
        axs[1].plot(sim_error[i,:].T, label="sim error")
        axs[1].legend()
    plt.show()

def parameter_rank_5(traindata, train_states, output_states, self_reference=False):

    for os in output_states:
        include_idx = []
        output_idx = train_states.index(os)
        evalout_idx = [output_idx]

        remain_states = train_states.copy()
        if not self_reference:
            # ensure none of the output state history is included if we don't self reference
            remain_states.remove(os)
            for i in range(1, 5):
                os_prev = os + "_%d" % i
                if os_prev in remain_states:
                    remain_states.remove(os_prev)
        else:
            # ensure /all/ of the output state history is included if we self reference
            include_idx.append(train_states.index(os))
            remain_states.remove(os)
            for i in range(1, 5):
                os_prev = os + "_%d" % i
                if os_prev in remain_states:
                    # print(os_prev, traindata[train_states.index(os_prev),:])
                    include_idx.append(train_states.index(os_prev))
                    remain_states.remove(os_prev)

        # min_rms = np.std(traindata[output_idx,:])
        min_rms = None

        while len(remain_states):
            for rs in remain_states:
                print("evaluating:", rs)
                r_idx = train_states.index(rs)
                evalin_idx = include_idx + [r_idx]

                A = solve(traindata, evalin_idx, evalout_idx)

                # direct solution with all current states known, how well does our fit estimate the next state?
                direct_est = A @ traindata[evalin_idx,:]
                # print("direct_est:", direct_est.shape, direct_est)
                direct_error = traindata[output_idx,1:] - direct_est[:,:-1]
                # print("direct_error:", direct_error.shape, direct_error)
                direct_rms = np.std(direct_error)
                print("direct_rms:", direct_rms)
                if min_rms is None or direct_rms < min_rms:
                    min_A = A
                    min_rms = direct_rms
                    min_idx = r_idx
                    min_est = direct_est
                    min_err = direct_error
                    min_evalin_idx = evalin_idx

                print("ERROR Direct:", os, "->", rs, rms(direct_error), "%.3f%%" % (100 * rms(direct_error) / rms(traindata[output_idx,1:])))
                # print("ERROR Sim:", output_states[i], rms(sim_error[i,:]), "%.3f%%" % (100 * rms(sim_error[i,:]) / rms(sim_est[i,:]) ))

            print(rms(min_err), rms(traindata[output_idx,1:]))
            print("Best next parameter:", train_states[min_idx], "rms val: %.05f" % min_rms,
                  "error = %.3f%%" % (100 * rms(min_err) / rms(traindata[output_idx,1:])))
            include_idx.append(min_idx)
            remain_states.remove(train_states[min_idx])

            if self_reference:
                sim_est = simulate(traindata, min_evalin_idx, evalout_idx, min_A)
                sim_error = traindata[output_idx,1:] - sim_est[:,:-1]

            terms = ""
            for i, idx in enumerate(min_evalin_idx):
                terms += "%.3f*" % min_A[0,i] + train_states[idx] + ", "
            print(os, "=", terms)

            fig, axs = plt.subplots(2, sharex=True)
            fig.suptitle("Estimate for: " + os + " = " + terms)
            axs[0].plot(traindata[output_idx,1:].T, label="original signal")
            if self_reference:
                axs[0].plot(sim_est[:,:-1].T, label="sim signal")
            else:
                axs[0].plot(min_est[:,:-1].T, label="fit signal")
            axs[0].legend()
            if self_reference:
                axs[1].plot(sim_error[0,:].T, label="sim error")
                y_mean = np.mean(sim_error[0,:])
                y_std = np.std(sim_error[0,:])
            else:
                axs[1].plot(min_err.T, label="fit error")
                y_mean = np.mean(min_err)
                y_std = np.std(min_err)
            print("  mean: %.4f" % y_mean, "std: %.4f" % y_std)
            # print(len(min_est[:,:-1].T))
            axs[1].hlines(y=y_mean-2*y_std, xmin=0, xmax=len(min_est[:,:-1].T), colors='green', linestyles='--')
            axs[1].hlines(y=y_mean+2*y_std, xmin=0, xmax=len(min_est[:,:-1].T), colors='green', linestyles='--', label="2*stddev")
            axs[1].legend()
            plt.show()

# evaluate each condition
for i, cond in enumerate(conditions):
    print(i, cond)
    condition_dict = { "condition": cond }
    traindata = np.array(train_data.cond_list[i]["traindata_list"]).T
    coeff = np.array(train_data.cond_list[i]["coeff"])

    if True:
        print("test pearson correlation coefficients:")
        print(traindata)
        corr = np.corrcoef(traindata)
        print("corr:\n", corr)

    # sysid = SystemIdentification(args.vehicle)
    # train_data.cond_list[i]["sysid"] = sysid

    if False:
        mass_solution_4(traindata, train_states, output_states, self_reference=True)

    if True:
        # reverse the data for doing NDI!
        traindata = np.fliplr(traindata)

    if True:
        parameter_rank_5(traindata, train_states, output_states, self_reference=False)
