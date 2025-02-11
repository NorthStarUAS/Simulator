from math import cos, sin
import numpy as np

from nstSimulator.utils.constants import d2r, g
from nstSimulator.sim.lib.props import att_node, fcs_node, imu_node

from .util import NotaPID

class pr_controller():
    def __init__(self):
        # envelope protection
        self.bank_limit_deg = 60.0

        # helpers
        self.roll_helper = NotaPID("roll", -45, 45, integral_gain=1.0, antiwindup=0.25, neutral_tolerance=0.02)
        self.yaw_helper = NotaPID("yaw", -20, 20, integral_gain=-0.01, antiwindup=0.25, neutral_tolerance=0.02)

        # integrators
        self.roll_int = 0.0
        self.yaw_int = 0.0

        # damper gains
        self.roll_damp_gain = 1500.0
        self.yaw_damp_gain = 1500.0

        # output
        self.aileron_cmd = 0.0
        self.rudder_cmd = 0.0

    # compute model-based roll and yaw commands to simultaneously achieve the
    # reference roll rate and yaw rate. This functions is fit from
    # the original flight data and involves a matrix inversion that is
    # precomputed offlin.  Here we use the inverted matrix directly and never
    # needs to be recomputed.
    def lat_func(self, ref_p, ref_beta):
        Ainv = np.array(
            [[5539.387453799963,  -656.7869385413367],
             [-630.2043681682369, 7844.231440517533]]
        )
        B = np.array(
            [[-0.18101905232004417, -0.005232046450801025, -0.00017122476763947896, 0.0012871295574104415, 4.112901593458797, -0.012910711892868918],
             [-0.28148143506417056, 0.0027324890386930005, -0.011315776036902089, 0.0026095125404917378, 7.031756136691342, 0.011047506105235635]]
        )
        x = np.array([ref_p, ref_beta])
        b = np.array([1, self.ay, self.gbody_y, self.vc_mps, 1/self.vc_mps, self.beta_deg])
        y = (Ainv @ x - B @ b) / self.qbar
        print("lon y:", y)
        return y.tolist()

    def update(self, roll_rate_request, yaw_rate_request):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getDouble("flying_confidence")
        phi_deg = att_node.getDouble("phi_deg")
        phi_rad = phi_deg * d2r
        theta_rad = att_node.getDouble("theta_deg") * d2r
        p_rps = imu_node.getDouble("p_rps")
        r_rps = imu_node.getDouble("r_rps")
        baseline_r = fcs_node.getDouble("baseline_r")
        self.ay = imu_node.getDouble("ay_mps2")
        gbody_y = sin(phi_rad) * cos(theta_rad) * g
        vc_mps = fcs_node.getDouble("vc_filt_mps")
        qbar = fcs_node.getDouble("qbar")
        beta_deg = fcs_node.getDouble("beta_deg")

        # envelope protection: bank angle limits
        max_p = (self.bank_limit_deg - phi_deg) * d2r * 0.5
        min_p = (-self.bank_limit_deg - phi_deg) * d2r * 0.5

        # Condition and limit the pilot requests
        ref_p = self.roll_helper.get_ref_value(roll_rate_request, 0, min_p, max_p, self.phi_deg, flying_confidence)
        ref_r = self.yaw_helper.get_ref_value(yaw_rate_cmd, baseline_r, None, None, 0, flying_confidence)

        # compute the direct surface position to achieve the command
        raw_roll_cmd, raw_yaw_cmd = self.lat_func(ref_p, ref_r)

        # run the integrators
        self.roll_int = self.roll_helper.integrator(ref_p, p_rps, flying_confidence)
        self.yaw_int = self.yaw_helper.integrator(ref_r, r_rps, flying_confidence)

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
