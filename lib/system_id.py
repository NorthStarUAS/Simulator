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
from scipy.optimize import least_squares, minimize
from scipy.sparse import lil_matrix

from lib.state_mgr import StateManager

class SystemIdentification():
    def __init__(self):
        self.traindata = []
        self.A = None
        self.model = {}
        self.state_mgr = StateManager()

    def add_state_vec(self, state_vec):
        self.traindata.append( state_vec )

    def opt_err(self, xk):
        A = xk.reshape( (self.states, self.states) )
        Yf = A @ self.X
        E = self.Y - Yf
        # generate a norm value per Y matrix column (per entry would
        # be better computationally but massively blows up our
        # available memory)
        error = np.linalg.norm(E, axis=0)
        return error

    def opt_compute_bounds(self, A, logical_bounds):
        # experiment with additing domain-knowledge constraints
        lower = np.array([-np.inf] * self.states**2)
        upper = np.array([np.inf] * self.states**2)

        for p1, p2, l, u in logical_bounds:
            row, col = self.state_mgr.get_state_index( [p1, p2] )
            n = self.states * row + col
            lower[n] = l
            upper[n] = u
            if A[row,col] < lower[n]:
                print("reseting A[%s,%s] from:" % (p1, p2), A[row,col], "to:", lower[n])
                A[row,col] = lower[n]
            if A[row,col] > upper[n]:
                print("reseting A[%s,%s] from:" % (p1, p2), A[row,col], "to:", upper[n])
                A[row,col] = upper[n]
        return A, lower, upper
    
    def opt_fit(self, A_guess=None):
        self.states = len(self.traindata[0])
        self.X = np.array(self.traindata[:-1]).T
        self.Y = np.array(self.traindata[1:]).T
        
        if A_guess is not None:
            A = A_guess.copy()
        else:
            A = np.eye(self.states)

        # experiment with additing domain-knowledge constraints
        logical_bounds = [
            ["airspeed", "elevator", 0, np.inf],
            ["airspeed", "q", -np.inf, 0],
            ["airspeed", "bax", 0, np.inf]
        ]
        A, lower, upper = self.opt_compute_bounds(A, logical_bounds)
        
        res = least_squares(self.opt_err, A.flatten(), bounds=(lower,upper), verbose=2)
        print( res["x"].reshape((self.states, self.states)) )
        return res["x"].reshape((self.states, self.states))
        
    def fit(self):
        states = len(self.traindata[0])
        self.X = np.array(self.traindata[:-1]).T
        self.Y = np.array(self.traindata[1:]).T
        print("X:\n", self.X.shape, np.array(self.X))
        print("Y:\n", self.Y.shape, np.array(self.Y))

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
        daX = da.from_array(self.X, chunks=(self.X.shape[0], 10000)).persist()
        u, s, vh = da.linalg.svd(daX)
        
        if True:
            # debug and sanity check
            print("u:\n", u.shape, u)
            print("s:\n", s.shape, s)
            print("vh:\n", vh.shape, vh)
            Xr = (u * s) @ vh[:states, :]
            print( "dask svd close?", np.allclose(self.X, Xr.compute()) )

        # after algebraic manipulation
        #
        # A = Y * V * D.inv() * U.T

        v = vh.T
        print("s inv:", (1/s).compute() )

        self.A = (self.Y @ (v[:,:states] * (1/s)) @ u.T).compute()
        print("A rank:", np.linalg.matrix_rank(self.A))
        print("A:\n", self.A.shape, self.A)

        if False:
            # for grins try an optimizer approach to see what happens,
            # this allows us to add constraints based on domain
            # knowledge, but it can also disrupt the delicate balance
            # in the feedback loop between dependent parameters, so ...
            self.A = self.opt_fit() # use I as initial guess
        
        # compute input state parameter ranges
        self.model["parameters"] = []
        for i in range(states):
            row = self.X[i,:]
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
