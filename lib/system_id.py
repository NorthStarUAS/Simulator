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
from matplotlib import pyplot as plt
import numpy as np

from lib.constants import r2d
from lib.state_mgr import StateManager

class SystemIdentification():
    def __init__(self, vehicle):
        self.traindata_list = []
        self.traindata = None
        self.A = None
        self.model = {}
        self.state_mgr = StateManager(vehicle)
        self.coeff = []

    def time_update(self, vehicle):
        if self.state_mgr.is_flying():
            # conditions -> here <-
            if abs(self.state_mgr.flaps - 0.0) < 0.1:
                self.state_mgr.compute_body_frame_values(self.state_mgr.have_alpha)
                state = self.state_mgr.gen_state_vector()
                #print(self.state_mgr.state2dict(state))
                self.traindata_list.append( state )
                if vehicle == "wing":
                    params = [ self.state_mgr.alpha*r2d, self.state_mgr.Cl, self.state_mgr.Cd, self.state_mgr.qbar,
                               self.state_mgr.accels[0], self.state_mgr.throttle ]
                    # print("params:", params)
                    self.coeff.append( params )

    def compute_lift_drag(self):
        if len(self.coeff):
            coeff = np.array(self.coeff)

            # fit a polynomial function for lift/drag coefficient curves
            from scipy.optimize import curve_fit

            def func(x, a, b, c):
                return a * x**2 + b * x + c

            plt.figure()
            plt.plot(coeff[:,0], coeff[:,1], ".", label="raw data")
            self.alpha_popt, pcov = curve_fit(func, coeff[:,0], coeff[:,1])
            plt.plot(coeff[:,0], func(coeff[:,0], *self.alpha_popt), '.', label='fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(self.alpha_popt))

            plt.figure()
            plt.plot(coeff[:,0], coeff[:,2], ".", label="raw data")
            self.drag_popt, pcov = curve_fit(func, coeff[:,0], coeff[:,2])
            plt.plot(coeff[:,0], func(coeff[:,0], *self.drag_popt), '.', label='fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(self.drag_popt))

            # alpha vs Cl, Cd plot
            num_bins = 100
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

            plt.figure()
            plt.plot(d1[:,0], d1[:,2], label="Cd vs. alpha (deg)")
            plt.xlabel("alpha (deg)")
            plt.legend()

            if False:
                # asi vs drag
                num_bins = 50
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

    def fit(self):
        self.traindata = np.array(self.traindata_list)

        if False:    # need to use filtfilt here to avoid phase change
            # signal smoothing experiment
            from scipy import signal
            idx_list = self.state_mgr.get_state_index( ["p", "q", "r"] )
            for i in idx_list:
                print(i)
                print(self.traindata[:,i].shape)
                sos = signal.butter(4, 15, 'low', fs=(1/self.state_mgr.dt),
                                    output='sos')
                filt = signal.sosfilt(sos, self.traindata[:,i]).astype(float)
                self.traindata[:,i] = filt

        output_list = self.state_mgr.get_state_index( self.state_mgr.output_states )
        print("output_list:", output_list)

        states = len(self.traindata[0])
        self.X = np.array(self.traindata[:-1]).T
        self.Y = np.array(self.traindata[1:,output_list]).T
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

        # compute expected ranges for output parameters
        self.model["parameters"] = []
        for i in range(states):
            row = self.X[i,:]
            min = np.min(row)
            max = np.max(row)
            mean = np.mean(row)
            if self.state_mgr.state_list[i] in self.state_mgr.internal_states:
                var_type = "internal"
            else:
                var_type = "output"
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

    def model_noise(self):
        # look at the frequency of the error terms in the output states which
        # suggest unmodeled effects such as short period oscillations and
        # turbulence, or artifacts in the data log (like a 5hz gps update rate
        # showing up in ekf velocity estimate.)

        output_index_list = self.state_mgr.get_state_index( self.state_mgr.output_states )
        pred = []
        for i in range(len(self.X.T)):
            v  = self.traindata[i,:].copy()
            p = self.A @ np.array(v)
            pred.append(p)
        Ypred = np.array(pred).T
        diff = Ypred - self.Y

        M=1024
        from scipy import signal
        for i in range(len(output_index_list)):
            # parameter: window='hann' may need to be added to the
            # signal.spectogram() call for older scipy/numpy versions that don't
            # yet agree on this name.
            freqs, times, Sx = signal.spectrogram(diff[i,:], fs=(1/self.state_mgr.dt),
                                                  nperseg=M, noverlap=M - 100,
                                                  detrend=False, scaling='spectrum')
            f, ax = plt.subplots()
            ax.pcolormesh(times, freqs, 10 * np.log10(Sx), cmap='viridis')
            ax.set_title(self.state_mgr.output_states[i] + " Spectogram")
            ax.set_ylabel('Frequency [Hz]')
            ax.set_xlabel('Time [s]');
            means = Sx.mean(axis=1)   # average each rows (by freq)
            num_bins = 15
            bins = np.linspace(0.5, num_bins+0.5, num_bins+1) # freq range
            #print(bins)

            bin_indices = np.digitize( freqs, bins )
            #print(bin_indices)
            d1 = []
            for j in range(num_bins):
                bin = j + 1
                total = np.sum(means[bin_indices==j+1])
                if not np.isnan(total):
                    pt = 0.5 * (bins[j] + bins[j+1])
                    #print( j, pt, total )
                    d1.append( [pt, total] )
            self.model["parameters"][output_index_list[i]]["noise"] = d1
            d1 = np.array(d1)
            plt.figure()
            plt.plot(d1[:,0], d1[:,1], label="freq vs energy")
            plt.xlabel("freq")
            plt.legend()
        plt.show()

    def analyze(self):
        output_index_list = self.state_mgr.get_state_index( self.state_mgr.output_states )
        states = len(self.traindata_list[0])
        params = self.model["parameters"]

        # report leading contributions towards computing each output state
        for i in range(len(self.state_mgr.output_states)):
            #print(self.state_names[i])
            row = self.A[i,:]
            energy = []
            for j in range(states):
                e = row[j] * (abs(params[j]["median"]) + 0.5 * abs(params[j]["std"])) * np.sign(params[j]["median"])
                # e = row[j] * (params[j]["max"] - params[j]["min"]) # probably no ...
                energy.append(e)
            idx = np.argsort(-np.abs(energy))
            total = np.sum(np.abs(energy))
            output_idx = output_index_list[i]
            params[output_idx]["formula"] = self.state_mgr.state_list[output_idx] + " = "
            formula = self.state_mgr.state_list[output_idx] + " = "
            first = True
            for j in idx:
                perc = 100 * energy[j] / total
                if abs(perc) < 0.01:
                    continue
                if first:
                    first = False
                else:
                    if perc >= 0:
                        formula += " + "
                    else:
                        formula += " - "
                formula += self.state_mgr.state_list[j] + " %.1f%%" % abs(perc)
            params[output_index_list[i]]["formula"] = formula
            print(params[output_index_list[i]]["formula"])

    def save(self, model_name, dt):
        # the median delta t from the data log is important to include
        # with the state transition matrix because the state
        # transition matrix coefficients assume this value for
        # realtime performance.

        self.model["dt"] = dt
        self.model["rows"] = len(self.state_mgr.output_states)
        self.model["cols"] = len(self.state_mgr.output_states) + len(self.state_mgr.internal_states)
        self.model["A"] = self.A.flatten().tolist()

        f = open(model_name, "w")
        json.dump(self.model, f, indent=4)
        f.close()
