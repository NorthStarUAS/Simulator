from math import cos, sin, tan
import numpy as np

from nstSimulator.utils.constants import d2r
from nstSimulator.sim.lib.props import att_node, fcs_node, imu_node

from .util import NotaPID

class q_controller():
    def __init__(self):
        # envelope protection
        self.alpha_limit_deg = 13.0
        self.vne_mps = 80
        theta_soft_limit = 15

        # helper
        self.pitch_helper = NotaPID("pitch", -theta_soft_limit, theta_soft_limit, integral_gain=-4.0, antiwindup=1.0, neutral_tolerance=0.03)

        # integrators
        self.pitch_int = 0.0

        # damper gains
        self.pitch_damp_gain = 2000.0

    # compute model-based pitch command to achieve the reference pitch rate.
    # This functions is fit from the original flight data and involves a matrix
    # inversion that is precomputed offlin.  Here we use the inverted matrix
    # directly and never needs to be recomputed.
    def lon_func(self, ref_q, qbar, aileron, rudder, ay, az):
        A = np.array(
            [[-1.79754412e+02, -2.05735090e+03, 8.66518876e-03, 1.08625576e-01, -1.39143682e+02, 1.05951379e+00]]
        )
        x = np.array([1, ref_q, abs(aileron)*qbar, abs(rudder)*qbar, abs(ay), az])
        y = (A @ x) / qbar
        # print("lon y:", y)
        return y[0]

    def update(self, pitch_rate_request):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getDouble("flying_confidence")
        theta_deg = att_node.getDouble("theta_deg")
        q_rps = imu_node.getDouble("q_rps")
        baseline_q = fcs_node.getDouble("baseline_q")
        ay = imu_node.getDouble("ay_mps2")
        az = imu_node.getDouble("az_mps2")
        qbar = fcs_node.getDouble("qbar")
        alpha_deg = fcs_node.getDouble("alpha_deg")
        aileron = fcs_node.getDouble("posAil_norm")
        rudder = fcs_node.getDouble("posRud_norm")

        # envelope protection (needs to move after or into the controller or at
        # least incorporate the ff term (and dampers?))  This must consider more
        # than just pitch rate and may need to lower the pitch angle hold value
        # simultaneously, however it takes time for speed to build up and alpha
        # to come down so how/where should the limited 'hold' value get set to?
        max_q = (self.alpha_limit_deg - alpha_deg) * d2r * 4
        # min_q = (self.vc_mps - self.vne_mps) * 0.1

        # Condition and limit the pilot request
        ref_q = self.pitch_helper.get_ref_value(pitch_rate_request, baseline_q, None, max_q, theta_deg, flying_confidence)

        # compute the direct surface position to achieve the command
        raw_pitch_cmd = self.lon_func(ref_q, qbar, aileron, rudder, ay, az)

        # run the integrators.
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
