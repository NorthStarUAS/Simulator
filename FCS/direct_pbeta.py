from math import cos, sin
import numpy as np

from lib.constants import d2r, gravity
from lib.props import accel_node, att_node, fcs_node, vel_node

from .util import NotaPID

class pbeta_controller():
    def __init__(self):
        # envelope protection
        self.phi_hard_limit_deg = 60.0
        phi_soft_limit_deg = 45.0
        beta_soft_limit_deg = 10.0

        # helpers
        self.roll_helper = NotaPID("roll", -phi_soft_limit_deg, phi_soft_limit_deg, integral_gain=1.0, antiwindup=0.25, neutral_tolerance=0.02)
        self.yaw_helper = NotaPID("yaw", -beta_soft_limit_deg,beta_soft_limit_deg, integral_gain=-0.01, antiwindup=0.25, neutral_tolerance=0.02)

        # integrators
        self.roll_int = 0.0
        self.yaw_int = 0.0

        # damper gains
        self.roll_damp_gain = 1500.0
        self.yaw_damp_gain = 6000.0

    # compute model-based roll and yaw commands to simultaneously achieve the
    # reference roll rate and beta (side-slip) angle. This functions is fit from
    # the original flight data and involves a matrix inversion that is
    # precomputed offlin.  Here we use the inverted matrix directly and never
    # needs to be recomputed.
    def lat_func(self, ref_p, ref_beta, qbar, ay, gbody_y, vc_mps):
        Ainv = np.array(
            [[5223.997719570232, 86.53137102359369],
            [3112.870284450966, -187.8833840322353]]
        )
        B = np.array(
            [[-0.3279732547932126, -0.006061380767969274, 0.0017838077680168345, 0.002582130232044947,   8.229002177507066],
            [11.381920691905997,   0.06423929309132188, -0.1514805151401035,   -0.10031783139998209, -318.79044889415076]]
        )
        x = np.array([ref_p, ref_beta])
        b = np.array([1, ay, gbody_y, vc_mps, 1/vc_mps])
        y = (Ainv @ x - B @ b) / qbar
        # print("lat y:", y)
        return y.tolist()

    def update(self, roll_rate_request, beta_deg_request):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getFloat("flying_confidence")
        phi_deg = att_node.getFloat("phi_deg")
        phi_rad = phi_deg * d2r
        theta_rad = att_node.getFloat("theta_deg") * d2r
        p_rps = vel_node.getFloat("p_rps")
        r_rps = vel_node.getFloat("r_rps")
        baseline_r = fcs_node.getFloat("baseline_r")
        self.ay = accel_node.getFloat("Ny") * gravity
        gbody_y = sin(phi_rad) * cos(theta_rad) * gravity
        vc_mps = fcs_node.getFloat("vc_filt_mps")
        qbar = fcs_node.getFloat("qbar")
        beta_deg = fcs_node.getFloat("beta_deg")

        # envelope protection: bank angle limits
        max_p = (self.phi_hard_limit_deg - phi_deg) * d2r * 0.5
        min_p = (-self.phi_hard_limit_deg - phi_deg) * d2r * 0.5

        # Condition and limit the pilot requests
        ref_p = self.roll_helper.get_ref_value(roll_rate_request, 0, min_p, max_p, phi_deg, flying_confidence)
        ref_beta = self.yaw_helper.get_ref_value(beta_deg_request, 0, None, None, 0, flying_confidence)

        # compute the direct surface position to achieve the command
        raw_roll_cmd, raw_yaw_cmd = self.lat_func(ref_p, ref_beta, qbar, self.ay, gbody_y, vc_mps)

        # run the integrators
        self.roll_int = self.roll_helper.integrator(ref_p, p_rps, flying_confidence)
        self.yaw_int = self.yaw_helper.integrator(ref_beta, beta_deg, flying_confidence)

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        roll_damp = p_rps * self.roll_damp_gain / qbar
        yaw_damp = (r_rps - baseline_r) * self.yaw_damp_gain / qbar

        # final output command
        roll_cmd = raw_roll_cmd + self.roll_int - roll_damp
        yaw_cmd = raw_yaw_cmd + self.yaw_int - yaw_damp
        # print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
        #       "raw ele: %.3f" % raw_elevator_cmd, "final ele: %.3f" % elevator_cmd)

        return roll_cmd, yaw_cmd
