from math import cos, sin, tan
import numpy as np

from nstSimulator.utils.constants import d2r, g
from nstSimulator.sim.lib.props import att_node, fcs_node, imu_node, vel_node

from .inertial_airdata_est import inv_alpha_func
from .pid import ap_pid_t
from .util import HoldOrPassThrough

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
        self.pid.Kp = 0.015
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

    # compute model-based pitch command to achieve the reference pitch rate.
    # This functions is fit from the original flight data and involves a matrix
    # inversion that is precomputed offlin.  Here we use the inverted matrix
    # directly and never needs to be recomputed.
    def lon_func(self, flaps_norm, ref_az, qbar):
        if qbar < 1:
            return 0                                       # no airspeed
        elif flaps_norm <= 0.25:
            A = np.array( [[ 0.03713305, 31.24626614 ]] )  # flaps up
        elif flaps_norm <= 0.75:
            A = np.array( [[ 0.06285491, 26.32095682 ]] )  # flaps 50%
        else:
            A = np.array( [[ 0.07604505, 23.9582486 ]] )   # flaps 100%
        x = np.array([1, ref_az/qbar])
        y = A @ x
        return y[0]

    def update(self, load_factor_request, dt):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getDouble("flying_confidence")
        flaps_norm = fcs_node.getDouble("posFlap_norm")
        theta_deg = att_node.getDouble("theta_deg")
        q_rps = imu_node.getDouble("q_rps")
        baseline_q = fcs_node.getDouble("baseline_q")
        baseline_lf = fcs_node.getDouble("baseline_lf")
        az = imu_node.getDouble("az_mps2")
        qbar = fcs_node.getDouble("qbar")

        # Condition and limit the pilot request
        ref_az = g * (1 + self.az_helper.get_ref_value(load_factor_request-1, baseline_lf-1, theta_deg, flying_confidence))
        #ref_az = az_request

        # envelope protection (needs to move after or into the controller or at
        # least incorporate the ff term (and dampers?))  This must consider more
        # than just pitch rate and may need to lower the pitch angle hold value
        # simultaneously, however it takes time for speed to build up and alpha
        # to come down so how/where should the limited 'hold' value get set to?
        min_lf = -1
        max_lf = 2.5
        # print("max/min az:", min_lf, max_lf)
        alpha_limit_deg = 13
        ep_max_az = inv_alpha_func(flaps_norm, alpha_limit_deg, qbar)
        print("env prot max az: %.3f  ref: %.3f" % (ep_max_az, ref_az))
        if ref_az < ep_max_az: ref_az = ep_max_az

        # model-based estimate of direct surface position to achieve the command
        model_pitch_cmd = self.lon_func(flaps_norm, ref_az, qbar)

        # pid controller accounts for model errors
        self.pid.Kp = 30 / qbar # gain schedule on qbar
        self.pid.max_integrator = 0.5 * flying_confidence
        pid_pitch_cmd = self.pid.update(dt, az, ref_az)

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        pitch_damp = (q_rps - baseline_q) * self.pitch_damp_gain / qbar

        # final output command
        pitch_cmd = model_pitch_cmd + pid_pitch_cmd + pitch_damp

        # print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
        #       "raw ele: %.3f" % raw_pitch_cmd, "final ele: %.3f" % pitch_cmd)
        # print("inc_az: %.2f" % az_request, "ref_az: %.2f" % ref_az, "raw ele: %.3f" % model_pitch_cmd, "final ele: %.3f" % pitch_cmd)

        return pitch_cmd
