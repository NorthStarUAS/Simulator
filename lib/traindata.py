from math import cos, pi, sin
import numpy as np
import os
import pickle
from tqdm import tqdm

from flightdata import flight_loader, flight_interp

from .constants import d2r, r2d, kt2mps
from .wind import Wind

class TrainData():
    def __init__(self):
        self.session = None
        self.cond_list = None

    def load_flightdata(self, path):
        self.session_file = path + ".session.pkl"

        if os.path.exists(self.session_file) and os.path.getmtime(self.session_file) > os.path.getmtime(path):
            print("loading session from cached pickle file ...")
            with open(self.session_file, "rb") as f:
                self.session = pickle.load(f)
                self.data = self.session["data"]
                self.flight_format = self.session["flight_format"]
                self.cond_list = self.session["cond_list"]
                if "train_states" in self.session:
                    self.train_states = self.session["train_states"]
                else:
                    self.train_states = None
            return

        self.data, self.flight_format = flight_loader.load(path)

        print("imu records:", len(self.data["imu"]))
        print("gps records:", len(self.data["gps"]))
        if "air" in self.data:
            print("airdata records:", len(self.data["air"]))
        if "act" in self.data:
            print("actuator records:", len(self.data["act"]))
        if len(self.data["imu"]) == 0 and len(self.data["gps"]) == 0:
            print("not enough data loaded to continue.")
            quit()

    def build(self, vehicle, invert_elevator, invert_rudder, state_mgr, conditions, train_states):
        if self.cond_list is not None and self.train_states == train_states:
            return

        self.train_states = train_states

        # dt estimation
        print("Estimating median dt from IMU records:")
        iter = flight_interp.IterateGroup(self.data)
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

        if False or self.flight_format != "cirrus_csv":
            # note: wind estimates are only needed for estimating alpha/beta (or
            # bvz/bvy) which is not needed if the aircraft is instrumented with
            # alpha/beta vanes and these are measured directly.

            from lib.wind2 import Wind2
            w2 = Wind2()
            pitot_scale, psi_bias, wn_interp, we_interp = w2.estimate( flight_interp.IterateGroup(self.data), imu_dt )

        # condition data collectors
        self.cond_list = []
        for i in range(len(conditions)):
            self.cond_list.append( { "traindata_list": [], "coeff": [] } )

        # iterate through the flight data log (a sequence of time samples of all the measured states)
        iter = flight_interp.IterateGroup(self.data)
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
                if vehicle == "wing":
                    state_mgr.set_throttle( actpt["throttle"] )
                    ail = actpt["aileron"]
                    ele = actpt["elevator"]
                    rud = actpt["rudder"]
                    if invert_elevator:
                        ele = -ele
                    if invert_rudder:
                        rud = -rud
                    if "flaps" in actpt:
                        flaps = actpt["flaps"]
                    else:
                        flaps = 0
                    state_mgr.set_flight_surfaces( ail, ele, rud, flaps )
                elif vehicle == "quad":
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
                if vehicle == "wing" or np.linalg.norm([navpt["vn"], navpt["ve"], navpt["vd"]]) > 0.000001:
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
                    self.cond_list[i]["traindata_list"].append( state )
                    if vehicle == "wing":
                        params = [ state_mgr.alpha*r2d, state_mgr.Cl_raw, 0, state_mgr.qbar,
                                    state_mgr.accels[0], state_mgr.throttle ]
                        # print("params:", params)
                        self.cond_list[i]["coeff"].append( params )

        # cache our work
        with open(self.session_file, "wb") as f:
            print("Saving a pickle cache of this session data...")
            session = {}
            session["data"] = self.data
            session["flight_format"] = self.flight_format
            session["cond_list"] = self.cond_list
            session["train_states"] = self.train_states
            pickle.dump(session, f)
