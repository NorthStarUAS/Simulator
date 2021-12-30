"""system identification

There is nothing new under the sun, but I haven't seen a system
identification process that is this easy with this much bang for the
buck myself.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

system id is a very broad topic that covers a multitude of approaches,
input/output parameters, system models, etc.

Here we implement a very specific approach to building a very specific
type of model.  The advantage is the approach is completely data
driven, very simple, very fast, and accurately approximates the
aircraft performance (versus some models that accurately estimate
specific aero parameters but the simulation may not perfom like the
real aircraft.)

The underlying "secret sauce" is we assemble a matrix X that is all
the v_i (state vectors) except the last one.  We make a matrix Y that
is all the v_i except the first one.)  X and Y are identical, except Y
is shifted over by one.

We write Y = A * X

If we can solve for A (in a least squares best fit senses) then we
have a state transition matrix that maps the set of current states to
the set of next states.

With this matrix we can predict the next state given any current
state.  This can be used for system integerity validataion, flight
simulation, possibly extracting traditional aero coefficients, and
possibly optimal flight control.

Note for fluids people: the problem setup is exactly the same as
performed with Dynamic Mode Decomposition (DMD), however in this
use-case we compute the fuil A matrix, whereas DMD is primarliy
interested in finding the leading eigenvalues and eigenvectors of the
A matrix, so that method employs a similarity transform to reduce the
computational effort.

"""

import dask.array as da         # dnf install python3-dask+array
import json
import numpy as np

from lib.state_mgr import StateManager

class SystemIdentification():
    def __init__(self):
        self.traindata = []
        self.A = None
        self.model = {}
        self.state_mgr = StateManager()

    def add_state_vec(self, state_vec):
        self.traindata.append( state_vec )
        
    def fit(self):
        states = len(self.traindata[0])
        X = np.array(self.traindata[:-1]).T
        Y = np.array(self.traindata[1:]).T
        print("X:\n", X.shape, np.array(X))
        print("Y:\n", Y.shape, np.array(Y))

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

        print("dask svd...")
        daX = da.from_array(X, chunks=(X.shape[0], 10000)).persist()
        u, s, vh = da.linalg.svd(daX)
        
        if True:
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
        print("s inv:", (1/s).compute() )

        self.A = (Y @ (v[:,:states] * (1/s)) @ u.T).compute()
        print("A rank:", np.linalg.matrix_rank(self.A))
        print("A:\n", self.A.shape, self.A)

        # compute input state parameter ranges
        self.model["parameters"] = []
        for i in range(states):
            row = X[i,:]
            min = np.min(row)
            max = np.max(row)
            mean = np.mean(row)
            if self.state_mgr.state_list[i] in self.state_mgr.ind_states:
                var_type = "independent"
            else:
                var_type = "dependent"
            self.model["parameters"].append(
                {
                    "name": self.state_mgr.state_list[i],
                    "min": np.min(row),
                    "max": np.max(row),
                    "median": np.median(row),
                    "std": np.std(row),
                    "type": var_type
                }
            )

    def analyze(self):
        states = len(self.traindata[0])
        params = self.model["parameters"]

        # report leading contributions towards computing each dependent state
        for i in range(states):
            if params[i]["type"] == "independent":
                continue
            #print(self.state_names[i])
            row = self.A[i,:]
            energy = []
            for j in range(states):
                e = row[j] * params[j]["std"]
                energy.append(e)
            idx = np.argsort(-np.abs(energy))
            total = np.sum(np.abs(energy))
            params[i]["formula"] = self.state_mgr.state_list[i] + " = "
            first = True
            for j in idx:
                perc = 100 * energy[j] / total
                if abs(perc) < 0.05:
                    continue
                if first:
                    first = False
                else:
                    if perc >= 0:
                        params[i]["formula"] += " + "
                    else:
                        params[i]["formula"] += " - "
                params[i]["formula"] += self.state_mgr.state_list[j] + " %.1f%%" % abs(perc)
            print(params[i]["formula"])
            
        # report leading contributions of each state to each dependent state
        for i in range(states):
            #print(self.state_names[i])
            col = self.A[:,i]
            energy = []
            for j in range(states):
                if params[j]["type"] == "independent":
                    e = 0
                else:
                    e = col[j] * params[i]["std"] / params[j]["std"]
                energy.append(e)
                #print(" ", self.state_mgr.state_list[i], self.state_mgr.state_list[j], col[j], e)
            idx = np.argsort(-np.abs(energy))
            #total = np.sum(np.abs(energy))
            params[i]["correlates to"] = ""
            first = True
            for j in idx:
                perc = 100 * energy[j]
                if abs(perc) < 0.0001:
                    continue
                if first:
                    first = False
                else:
                    params[i]["correlates to"] += ", "
                params[i]["correlates to"] += self.state_mgr.state_list[j] + ": "
                if perc >= 0:
                    params[i]["correlates to"] += "+"
                else:
                    params[i]["correlates to"] += "-"
                params[i]["correlates to"] += "%.3f%%" % abs(perc)
            print(self.state_mgr.state_list[i], "correlates to:", params[i]["correlates to"])
                
    def save(self, model_name, dt):
        # the median delta t from the data log is important to include
        # with the state transition matrix because the state
        # transition matrix coefficients assume this value for
        # realtime performance.
        
        self.model["dt"] = dt
        self.model["A"] = self.A.tolist()

        f = open(model_name, "w")
        json.dump(self.model, f, indent=4)
        f.close()
