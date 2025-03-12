# Notes:
#
# The controller model fit to original flight data doesn't match the sim model
# as well as I'd like.  At least a part of this is issues with the elevator trim
# point of the sim model (not matching original flight test data.)  The is
# certainly additional errors/limits to the simple controller model fit.
#
# The PID at the end should suck up this error.  However, the pid gains have to
# be kept pretty low to avoid oscillations.  Dialing up the gains on the pid for
# better response doesn't work.
#
# So the end result is the controller is pretty responsive up to the limits of
# the controller model fit and super lazy outside of that.  In this case asking
# for max load factor means you get part way there really quickly, and the rest
# takes forever to get to.
#
# This could mean that controller model has a pretty good fit around the usual
# envelope, but not at the limits.  The relationship should be pretty linear so
# throwing a huge chunk of usual flight data at the fit may not be capturing the
# correct slope due to not enough data out at the limits to fit to (versus the
# noise in the middle.)

import matplotlib.pyplot as plt
plt.switch_backend('TkAgg')

import numpy as np
from pathlib import Path

import pickle

from nstSimulator.fcs.binned_fit import IntervalBinnedFit
from nstSimulator.fcs.hold import HoldOrPassThrough
from nstSimulator.fcs.pid import ap_pid_t
from nstSimulator.sim.lib.props import att_node, fcs_node, imu_node
from nstSimulator.utils.constants import g

from .inertial_airdata_est import inv_alpha_func

# az/qbar (biggest when az is biggest and qbar is smallest)
# az range = 2.5*g to -1.5*g
# vc range ~= 60 - 200 kts ~= 30 - 100 mps ~= 506.25 - 5625 qbar
# min az/qbar = -24.525/506.25 = -0.048444
# max az/qbar =  14.715/506.25 =  0.029067

class nzthe_controller():
    def __init__(self):
        # envelope protection
        self.alpha_limit_deg = 13.0
        self.vne_mps = 80
        min_theta_hold = -15
        max_theta_hold = 15
        # note: lf (load factor) is shifted by -1 when calling get_ref_value()
        # so that zero inceptor = zero (lf-1).  The returned ref value is
        # shifted back to real lf/az before continuing on.
        self.min_lf = -1
        self.max_lf = 2.5

        # pid
        self.pid = ap_pid_t("pitch")
        self.pid.Kp = 0.05
        self.pid.Td = 0.0
        self.pid.Ti = 4.0
        self.pid.u_min = -1
        self.pid.u_max = 1
        self.pid.debug = False
        self.pid.enable = True

        # helper
        self.az_helper = HoldOrPassThrough("pitch", min_theta_hold, max_theta_hold, self.min_lf-1, self.max_lf-1, neutral_tolerance=0.03, hold_gain=0.1, debug=True)

        # damper gains
        self.pitch_damp_gain = 1000

        # flag to help diminish and work-around envelope protection for takeoff
        self.on_ground = True

        # on-the-fly model building experiment
        file = Path("ltf-pitch.pickle")
        if file.is_file():
            f = open(file, "rb")
            ltf = pickle.load(f)
            f.close()
            self.flap0_func = ltf[0]
            self.flap50_func = ltf[1]
            self.flap100_func = ltf[2]

            plt.figure()
            plt.plot(self.flap0_func.xp, self.flap0_func.fp, label="flaps 0")
            xs = np.linspace(np.min(self.flap0_func.xp), np.max(self.flap0_func.xp), 10)
            ys = xs * self.flap0_func.fit_model[0] + self.flap0_func.fit_model[1]
            plt.plot(xs, ys, label="fit")
            plt.legend()

            plt.figure()
            plt.plot(self.flap50_func.xp, self.flap50_func.fp, label="flaps 50")
            xs = np.linspace(np.min(self.flap50_func.xp), np.max(self.flap50_func.xp), 10)
            ys = xs * self.flap50_func.fit_model[0] + self.flap50_func.fit_model[1]
            plt.plot(xs, ys, label="fit")
            plt.legend()

            plt.figure()
            plt.plot(self.flap100_func.xp, self.flap100_func.fp, label="flaps 100")
            xs = np.linspace(np.min(self.flap100_func.xp), np.max(self.flap100_func.xp), 10)
            ys = xs * self.flap100_func.fit_model[0] + self.flap100_func.fit_model[1]
            plt.plot(xs, ys, label="fit")
            plt.legend()

            plt.show()
        else:
            self.flap0_func = IntervalBinnedFit(0.002)
            self.flap50_func = IntervalBinnedFit(0.002)
            self.flap100_func = IntervalBinnedFit(0.002)
        self.pickle_timer = 0

    # compute model-based pitch command to achieve the reference load factor.
    # This functions is fit from the original flight data and involves a matrix
    # inversion that is precomputed offline.
    def lon_func(self, flaps_norm, ref_az, qbar):
        if qbar < 1:
            return 0                                       # no airspeed
        elif flaps_norm <= 0.25:
            A = np.array( [[ 0.03526925, 30.79822387]] )  # flaps up  (updated 2/28/2025 with unfiltered raw data)
        elif flaps_norm <= 0.75:
            A = np.array( [[ 0.059158,   25.59681645]] )  # flaps 50%
        else:
            A = np.array( [[ 0.07080293, 23.44809481]] )  # flaps 100%
        x = np.array([1, ref_az/qbar])
        y = A @ x
        return y[0]

    def fit_func_update(self, flaps_norm, az, qbar, elev_norm, dt):
        print("update qbar:", qbar, az, flaps_norm, dt, elev_norm)
        if qbar < 1:
            y = 0                                       # no airspeed
        elif flaps_norm <= 0.25:
            self.flap0_func.AddData(az/qbar, elev_norm, dt)
        elif flaps_norm <= 0.75:
            self.flap50_func.AddData(az/qbar, elev_norm, dt)
        else:
            self.flap100_func.AddData(az/qbar, elev_norm, dt)

        # save every 60 seconds
        self.pickle_timer += dt
        if self.pickle_timer >= 60:
            self.pickle_timer = 0
            ltf = [ self.flap0_func, self.flap50_func, self.flap100_func ]
            file = open("ltf-pitch.pickle", "wb")
            pickle.dump(ltf, file)
            file.close()

    def fit_func_interp(self, flaps_norm, ref_az, qbar):
        if qbar < 1:
            y = 0                                       # no airspeed
        elif flaps_norm <= 0.25:
            y = self.flap0_func.FitModel()
            y = self.flap0_func.Interp(ref_az/qbar)
        elif flaps_norm <= 0.75:
            y = self.flap50_func.FitModel()
            y = self.flap50_func.Interp(ref_az/qbar)
        else:
            y = self.flap100_func.FitModel()
            y = self.flap100_func.Interp(ref_az/qbar)
        return y

    def update(self, load_factor_request, dt):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getDouble("flying_confidence")
        elev_norm = fcs_node.getDouble("posElev_norm")
        flaps_norm = fcs_node.getDouble("posFlap_norm")
        theta_deg = att_node.getDouble("theta_deg")
        q_rps = imu_node.getDouble("q_rps")
        baseline_q = fcs_node.getDouble("baseline_q")
        baseline_lf = fcs_node.getDouble("baseline_lf")
        az = imu_node.getDouble("az_mps2")
        # vc_mps = vel_node.getDouble("vc_mps")
        qbar_filt = fcs_node.getDouble("qbar_filt")

        # Condition and limit the pilot request
        ref_az = g * (1 + self.az_helper.get_ref_value(load_factor_request-1, baseline_lf-1, theta_deg, flying_confidence))
        #ref_az = az_request

        # envelope protection (doesn't protect against a 'zoom' maneuver for
        # lack of a better name for it)
        if flying_confidence < 0.1:
            self.on_ground = True
        elif flying_confidence > 0.9:
            self.on_ground = False
        alpha_limit_deg = 13
        ep_max_az = inv_alpha_func(flaps_norm, alpha_limit_deg, qbar_filt)
        print("env prot max az: %.3f  ref: %.3f" % (ep_max_az, ref_az))
        if self.on_ground:
            # no envelope protection for pitch/az/speed/alpha
            pass
        else:
            # flying for sure, so turn on envelope protection
            if ref_az < ep_max_az: ref_az = ep_max_az

        # model-based estimate of direct surface position to achieve the command
        model_pitch_cmd = self.lon_func(flaps_norm, ref_az, qbar_filt)
        if not self.on_ground:
            self.fit_func_update(flaps_norm, ref_az, qbar_filt, elev_norm, dt)
        fit_pitch_cmd = self.fit_func_interp(flaps_norm, ref_az, qbar_filt)
        print("mdl: %.2f  fit: %.2f" % (model_pitch_cmd, fit_pitch_cmd))

        # hack to work around simulation model error
        model_pitch_cmd -= 225 / qbar_filt

        # pid controller accounts for model errors

        # gain schedule on qbar (needs to be schedule on qbar, I check vc and no scheduling just to be sure)
        self.pid.Kp = 30 / qbar_filt

        self.pid.max_integrator = 0.75 * flying_confidence
        pid_pitch_cmd = self.pid.update(dt, az, ref_az)

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        pitch_damp = (q_rps - baseline_q) * self.pitch_damp_gain / qbar_filt

        # final output command
        # pitch_cmd = model_pitch_cmd + pid_pitch_cmd + pitch_damp
        pitch_cmd = fit_pitch_cmd + pid_pitch_cmd + pitch_damp
        # pitch_cmd = pid_pitch_cmd + pitch_damp # ignore model, just pid and damp

        # print("inc_az: %.2f" % (load_factor_request*g), "ref_az: %.2f" % ref_az, "act az: %.2f" % az)
        # print("  mdl ele: %.2f" % model_pitch_cmd, "pid: %.2f" % pid_pitch_cmd, "damp: %.3f" % pitch_damp, "final ele: %.3f" % pitch_cmd)

        return pitch_cmd
