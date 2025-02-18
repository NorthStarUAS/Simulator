from math import cos, sin, tan
import numpy as np

from nstSimulator.utils.constants import d2r, r2d
from nstSimulator.sim.lib.props import att_node, fcs_node, imu_node

class r_controller():
    def __init__(self):
        # damper gains
        self.yaw_damp_gain = 2000.0

    # compute model-based pitch command to achieve the reference pitch rate.
    # This functions is fit from the original flight data and involves a matrix
    # inversion that is precomputed offlin.  Here we use the inverted matrix
    # directly and never needs to be recomputed.
    def lat_func(self, ref_r, qbar, ay):
        A = np.array(
            # [[  -74.89261715, -4962.21757703,   533.26172084]]
            [[  -74.89261715, -4962.21757703,   0]]
        )
        x = np.array([1, ref_r, ay])
        y = (A @ x) / qbar
        # print("lon y:", y)
        return y[0]

    def update(self, yaw_rate_request):
        # fetch and compute all the values needed by the control laws
        flying_confidence = fcs_node.getDouble("flying_confidence")
        r_rps = imu_node.getDouble("r_rps")
        baseline_r = fcs_node.getDouble("baseline_r")
        ay = imu_node.getDouble("ay_mps2")
        qbar = fcs_node.getDouble("qbar")

        # Condition and limit the pilot requests
        # ref_r = self.yaw_helper.get_ref_value(yaw_rate_request, baseline_r, min_r, max_r, psi_deg, flying_confidence)
        ref_r = yaw_rate_request + baseline_r

        # compute the direct surface position to achieve the command
        raw_yaw_cmd = self.lat_func(ref_r, qbar, ay)
        # print("roll_cmd:", raw_roll_cmd)

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        yaw_damp = (r_rps - baseline_r) * self.yaw_damp_gain / qbar

        # final output command
        yaw_cmd = raw_yaw_cmd - yaw_damp
        # print("inc_r: %.3f" % yaw_rate_request, "bl_r: %.3f" % baseline_r, "ref_r: %.3f" % ref_r,
        #       "raw rud: %.3f" % raw_yaw_cmd, "final rud: %.3f" % yaw_cmd)

        return yaw_cmd
