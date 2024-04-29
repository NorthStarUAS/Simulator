from math import cos, sin, tan
import numpy as np

from lib.constants import d2r, gravity
from lib.props import accel_node, att_node, fcs_node, vel_node

from .util import NotaPID

class q_controller():
    def __init__(self):
        # envelope protection
        self.alpha_limit_deg = 13.0
        self.vne_mps = 80
        theta_soft_limit = 15

        # helper
        self.pitch_helper = NotaPID("pitch", -theta_soft_limit, theta_soft_limit, integral_gain=-4.0, antiwindup=0.5, neutral_tolerance=0.03)

        # integrators
        self.pitch_int = 0.0

        # damper gains
        self.pitch_damp_gain = 1500.0

    # compute model-based pitch command to achieve the reference pitch rate.
    # This functions is fit from the original flight data and involves a matrix
    # inversion that is precomputed offlin.  Here we use the inverted matrix
    # directly and never needs to be recomputed.
    def lon_func(self, ref_q, qbar, ay, gbody_y, vc_mps):
        Ainv = np.array(
            [[-4996.77049111088]]
        )
        B = np.array(
            [[0.15640149796443698, -0.00043212705017340664, 0.01596103011849002, -0.00017520759288595494, -0.0016056595485786098, -5.957540570227146]]
        )
        x = np.array([ref_q])
        b = np.array([1, ay, abs(ay), gbody_y, vc_mps, 1/vc_mps])
        y = (Ainv @ x - B @ b) / qbar
        # print("lon y:", y)
        return y[0]

    def update(self, pitch_rate_request):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getFloat("flying_confidence")
        phi_rad = att_node.getFloat("phi_deg") * d2r
        theta_deg = att_node.getFloat("theta_deg")
        theta_rad = theta_deg * d2r
        q_rps = vel_node.getFloat("q_rps")
        baseline_q = fcs_node.getFloat("baseline_q")
        ay = accel_node.getFloat("Ny") * gravity
        gbody_y = sin(phi_rad) * cos(theta_rad) * gravity
        vc_mps = fcs_node.getFloat("vc_filt_mps")
        qbar = fcs_node.getFloat("qbar")
        alpha_deg = fcs_node.getFloat("alpha_deg")

        # envelope protection (needs to move after or into the controller or at
        # least incorporate the ff term (and dampers?))  This must consider more
        # than just pitch rate and may need to lower the pitch angle hold value
        # simultaneously, however it takes time for speed to build up and alpha
        # to come down so how/where should the limited 'hold' value get set to?
        max_q = (self.alpha_limit_deg - alpha_deg) * d2r * 2
        # min_q = (self.vc_mps - self.vne_mps) * 0.1

        # Condition and limit the pilot request
        ref_q = self.pitch_helper.get_ref_value(pitch_rate_request, baseline_q, None, max_q, theta_deg, flying_confidence)

        # compute the direct surface position to achieve the command
        raw_pitch_cmd = self.lon_func(ref_q, qbar, ay, gbody_y, vc_mps)

        # run the integrators.  Tip of the hat to imperfect models vs the real
        # world.  The integrators suck up any difference between the model and
        # the real aircraft. Imperfect models can be due to linear fit limits,
        # change in aircraft weight and balance, change in atmospheric
        # conditions, etc.
        self.pitch_int = self.pitch_helper.integrator(ref_q, q_rps, flying_confidence)
        # print("pitch integrators: %.2f %.2f %.2f" % (aileron_int, self.pitch_int, rudder_int))  # move outside

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        pitch_damp = (q_rps - baseline_q) * self.pitch_damp_gain / qbar

        # final output command
        pitch_cmd = raw_pitch_cmd + self.pitch_int + pitch_damp
        # print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
        #       "raw ele: %.3f" % raw_pitch_cmd, "final ele: %.3f" % pitch_cmd)

        return pitch_cmd
