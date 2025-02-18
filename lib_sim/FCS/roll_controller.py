from math import cos, sin, tan
import numpy as np

from nstSimulator.utils.constants import d2r
from nstSimulator.sim.lib.props import att_node, fcs_node, imu_node

from .pid import ap_pid_t
from .util import HoldOrPassThrough

class p_controller():
    def __init__(self):
        # envelope protection
        self.phi_hard_limit_deg = 60.0
        phi_hold_limit_deg = 45.0
        p_rate_limit_rps = 40 * d2r

        # pid
        self.pid = ap_pid_t("roll")
        self.pid.Kp = 3.0
        self.pid.Td = 0.0
        self.pid.Ti = 3.0
        self.pid.u_min = -1
        self.pid.u_max = 1
        self.pid.debug = False
        self.pid.enable = True

        # helper
        self.roll_helper = HoldOrPassThrough("roll", -phi_hold_limit_deg, phi_hold_limit_deg, -p_rate_limit_rps, p_rate_limit_rps, neutral_tolerance=0.02, hold_gain=0.05)

        # damper gains
        self.roll_damp_gain = 2000.0

    # compute model-based pitch command to achieve the reference pitch rate.
    # This functions is fit from the original flight data and involves a matrix
    # inversion that is precomputed offlin.  Here we use the inverted matrix
    # directly and never needs to be recomputed.
    def lat_func(self, ref_p, qbar, elevator, rudder, ay, az):
        A = np.array(
            [[-8.05125235e+01, 4.28489185e+03, 4.30885143e-02, -2.18118187e-01, -2.61807779e+02, -2.49792949e+01]]
        )
        x = np.array([1, ref_p, elevator*qbar, rudder*qbar, ay, az])
        y = (A @ x) / qbar
        # print("lon y:", y)
        return y[0]

    def update(self, roll_rate_request, dt):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getDouble("flying_confidence")
        phi_deg = att_node.getDouble("phi_deg")
        p_rps = imu_node.getDouble("p_rps")
        ay = imu_node.getDouble("ay_mps2")
        az = imu_node.getDouble("az_mps2")
        qbar = fcs_node.getDouble("qbar")
        elevator = fcs_node.getDouble("posElev_norm")
        rudder = fcs_node.getDouble("posRud_norm")

        # Condition and limit the pilot requests
        ref_p = self.roll_helper.get_ref_value(roll_rate_request, 0, phi_deg, flying_confidence)

        # envelope protection: enforce hard bank angle limits through additional
        # roll rate limits
        max_p = (self.phi_hard_limit_deg - phi_deg) * d2r * 1.0
        min_p = (-self.phi_hard_limit_deg - phi_deg) * d2r * 1.0
        if ref_p < min_p: ref_p = min_p
        if ref_p > max_p: ref_p = max_p

        # model-based estimate of direct surface position to achieve the command
        model_roll_cmd = self.lat_func(ref_p, qbar, elevator, rudder, ay, az)

        # pid controller accounts for model errors
        self.pid.Kp = 3000 / qbar # gain schedule on qbar
        self.pid.max_integrator = 0.1 * flying_confidence
        pid_roll_cmd = self.pid.update(dt, p_rps, ref_p)

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        roll_damp = p_rps * self.roll_damp_gain / qbar

        # final output command
        roll_cmd = model_roll_cmd + pid_roll_cmd - roll_damp

        # print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
        #       "raw ele: %.3f" % raw_elevator_cmd, "final ele: %.3f" % elevator_cmd)

        return roll_cmd
