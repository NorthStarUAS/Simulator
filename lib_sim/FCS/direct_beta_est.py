from math import cos, sin, tan
import numpy as np

from nstSimulator.utils.constants import d2r, r2d
from nstSimulator.sim.lib.props import att_node, fcs_node, imu_node

from .util import NotaPID

class beta_est_controller():
    def __init__(self):
        # envelope protection
        self.beta_hard_limit_deg = 20.0
        beta_soft_limit_deg = 10.0

        # helper
        self.beta_helper = NotaPID("beta", -beta_soft_limit_deg, beta_soft_limit_deg, integral_gain=1.0, antiwindup=0.5, neutral_tolerance=0.02)

        # integrators
        self.beta_int = 0.0

        # damper gains
        self.beta_damp_gain = 2000.0

    # compute model-based pitch command to achieve the reference pitch rate.
    # This functions is fit from the original flight data and involves a matrix
    # inversion that is precomputed offlin.  Here we use the inverted matrix
    # directly and never needs to be recomputed.
    def lat_func(self, ref_beta, qbar, elevator, rudder, ay, az):
        A = np.array(
            [[-8.05125235e+01, 4.28489185e+03, 4.30885143e-02, -2.18118187e-01, -2.61807779e+02, -2.49792949e+01]]
        )
        print("")
        x = np.array([1, ref_beta, elevator*qbar, rudder*qbar, ay, az])
        y = (A @ x) / qbar
        # print("lon y:", y)
        return y[0]

    def update(self, roll_rate_request):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getDouble("flying_confidence")
        phi_deg = att_node.getDouble("phi_deg")
        p_rps = imu_node.getDouble("p_rps")
        ay = imu_node.getDouble("ay_mps2")
        az = imu_node.getDouble("az_mps2")
        qbar = fcs_node.getDouble("qbar")
        elevator = fcs_node.getDouble("posElev_norm")
        rudder = fcs_node.getDouble("posRud_norm")

        # envelope protection: bank angle limits
        max_p = (self.phi_hard_limit_deg - phi_deg) * d2r * 0.5
        min_p = (-self.phi_hard_limit_deg - phi_deg) * d2r * 0.5

        # Condition and limit the pilot requests
        ref_p = self.roll_helper.get_ref_value(roll_rate_request, 0, min_p, max_p, phi_deg, flying_confidence)

        # compute the direct surface position to achieve the command
        raw_roll_cmd = self.lat_func(ref_p, qbar, elevator, rudder, ay, az)
        # print("roll_cmd:", raw_roll_cmd)

        # run the integrators
        self.roll_int = self.roll_helper.integrator(ref_p, p_rps, flying_confidence)
        print("(dps) ref: %.2f  act: %.2f" % (ref_p*r2d, p_rps*r2d))

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        roll_damp = p_rps * self.roll_damp_gain / qbar

        # final output command
        roll_cmd = raw_roll_cmd + self.roll_int - roll_damp
        # print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
        #       "raw ele: %.3f" % raw_elevator_cmd, "final ele: %.3f" % elevator_cmd)

        return roll_cmd
