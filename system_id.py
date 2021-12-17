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

class SystemIdentification():
    def __init__(self):
        self.traindata = []
        self.A = None
        self.k = 0
        self.model = {}

    def add_state_vec(self, state_vec):
        self.traindata.append( state_vec )
        
    def fit(self, k, state_names):
        states = len(self.traindata[0])
        data1 = []
        self.k = k              # add this many previous states
        for i in range(self.k, len(self.traindata)):
            v = list(self.traindata[i])
            for j in range(1, self.k+1):
                v.extend(self.traindata[i-j])
            #print(v)
            data1.append(v)

        X = np.array(data1[:-1]).T
        Y = np.array(self.traindata[1+self.k:]).T
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
        print("u:\n", u.shape, u)
        print("s:\n", s.shape, s)
        print("vh:\n", vh.shape, vh)
        Xr = (u * s) @ vh[:states*(self.k+1), :]
        #print( "dask close?", np.allclose(X, Xr.compute()) )

        # after algebraic manipulation
        #
        # A = Y * V * D.inv() * U.T

        v = vh.T
        print("s inv:", (1/s).compute() )

        self.A = (Y @ (v[:,:states*(self.k+1)] * (1/s)) @ u.T).compute()
        print("A rank:", np.linalg.matrix_rank(self.A))
        print("A:\n", self.A.shape, self.A)

        # compute input state parameter ranges
        self.model["parameters"] = []
        for i in range(states):
            row = X[i,:]
            min = np.min(row)
            max = np.max(row)
            mean = np.mean(row)
            self.model["parameters"].append( { "name": state_names[i],
                                               "min": np.min(row),
                                               "max": np.max(row),
                                               "median": np.median(row) } )

    def save(self, model_name, dt):
        # we ask for the data delta t at this point to save it with
        # the state transition matrix, A.  Later it's important to
        # know the input data dt when estimating future states for
        # simulation, system integrity, flight control, etc.
        
        self.model["dt"] = dt
        self.model["k"] = self.k
        self.model["A"] = self.A.tolist()

        f = open(model_name, "w")
        json.dump(self.model, f, indent=4)
        f.close()
