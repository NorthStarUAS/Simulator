from math import cos, pi, sin, tan
from lib.constants import d2r, gravity
from lib.props import aero_node, att_node, control_engine_node, control_flight_node, fcs_node, inceptor_node, vel_node

from FCS.direct_pbeta import pbeta_controller
from FCS.direct_q import q_controller
from FCS.util import IsFlying

rho = 1.225

# a simple alpha estimator fit from flight test data
def alpha_func(qbar, az, p, q, ax):
    p = 0 # roll rate shows up in our alpha measurement because the alpha vane is at the end of the wing, but let's zero it and ignore it.
    alpha_deg = -6.3792 + 14993.7058/qbar -0.3121*az - 4.3545*p + 5.3980*q + 0.2199*ax
    return alpha_deg

# a simple beta estimator fit from flight test data (todo: fit a beta function without rudder)
def beta_func(qbar, ay, r):
    rudder_cmd = inceptor_node.getFloat("rudder")
    throttle_cmd = inceptor_node.getFloat("throttle")
    beta_deg = -0.3552 - 12.1898*rudder_cmd - 3.5411*ay + 7.1957*r + 0.0008*ay*qbar + 0.9769*throttle_cmd
    return beta_deg

class FCSMgr():
    def __init__(self):
        # stick -> rate command scaling
        self.roll_stick_scale = 30 * d2r   # rad
        self.pitch_stick_scale = 30 * d2r  # rad
        self.yaw_stick_scale = 20          # maps to beta_deg

        self.fcs_lat = pbeta_controller()
        self.fcs_lon = q_controller()
        self.is_flying = IsFlying(on_ground_for_sure_mps=30, flying_for_sure_mps=40)

        # filtered state (clamp to minimum of 25 mps because we need to divide
        # by airspeed and qbar so this must be definitely positive 100% of the time.)
        self.vc_mps = 25
        self.vtrue_mps = 25

    def compute_stuff(self):
        # update state and filters

        # velocity terms
        vc_mps = vel_node.getFloat("vc_mps")
        if vc_mps < 25: vc_mps = 25
        self.vc_mps = 0.99 * self.vc_mps + 0.01 * vc_mps
        vtrue_mps = vel_node.getFloat("vtrue_mps")
        if vtrue_mps < 25: vtrue_mps = 25
        self.vtrue_mps = 0.99 * self.vtrue_mps + 0.01 * vtrue_mps
        qbar = 0.5 * self.vc_mps**2 * rho
        fcs_node.setFloat("vc_filt_mps", self.vc_mps)
        fcs_node.setFloat("vtrue_filt_mps", self.vtrue_mps)
        fcs_node.setFloat("qbar", qbar)

        # in the air vs on the ground?  (uses a sigmoid function between
        # threshold speeds)
        flying_confidence = self.is_flying.get_flying_confidence(self.vc_mps)
        fcs_node.setFloat("flying_confidence", flying_confidence)

        # alpha / beta estimates (or direct from sim model)
        if flying_confidence > 0.5:
            if True:
                # sensed directly (or from sim model)
                alpha_deg = aero_node.getFloat("alpha_deg")
                beta_deg = aero_node.getFloat("beta_deg")
            else:
                # inertial+airdata estimate (behaves very wrong at low airspeeds, ok in flight!)
                self.alpha_deg = self.alpha_func(qbar, az, p, q, ax)
                self.beta_deg = self.beta_func(qbar, ay, r)  # this functions drifts and can get stuck!
        else:
            alpha_deg = att_node.getFloat("theta_deg")
            beta_deg = 0

        # Feed forward steady state q and r basd on bank angle/turn rate.
        # Presuming a steady state level turn, compute turn rate =
        # func(velocity, bank angle).  This is the one feed forward term used in
        # this set of control laws and it is purely physics based and works for
        # all fixed wing aircraft.
        phi_rad = att_node.getFloat("phi_deg") * d2r
        if abs(phi_rad) < pi * 0.5 * 0.9:
            turn_rate_rps = tan(phi_rad) * -gravity / vtrue_mps
        else:
            turn_rate_rps = 0
        # compute a baseline q and r for the presumed steady state level turn,
        # this is what we dampen towards
        baseline_q = sin(phi_rad) * turn_rate_rps
        baseline_r = cos(phi_rad) * turn_rate_rps
        # print("tr: %.3f" % turn_rate_rps, "q: %.3f %.3f" % (baseline_q, self.q), "r: %.3f %.3f" % (baseline_r, self.r))
        fcs_node.setFloat("baseline_q", baseline_q)
        fcs_node.setFloat("baseline_r", baseline_r)

    def update(self):
        # update state and filters
        self.compute_stuff()

        # Pilot commands
        roll_rate_request = inceptor_node.getFloat("aileron") * self.roll_stick_scale
        pitch_rate_request = -inceptor_node.getFloat("elevator") * self.pitch_stick_scale
        beta_deg_request = -inceptor_node.getFloat("rudder") * self.yaw_stick_scale

        roll_cmd, yaw_cmd = self.fcs_lat.update(roll_rate_request, beta_deg_request)
        pitch_cmd = self.fcs_lon.update(pitch_rate_request)
        print("integrators: %.2f %.2f %.2f" % (self.fcs_lat.roll_int, self.fcs_lon.pitch_int, self.fcs_lat.yaw_int))
        control_flight_node.setFloat("aileron", roll_cmd)
        control_flight_node.setFloat("rudder", yaw_cmd)
        control_flight_node.setFloat("elevator", pitch_cmd)

        # pass through flaps and throttle for now
        control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
        control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))
        throttle_cmd = inceptor_node.getFloat("throttle")
        control_engine_node.setFloat("throttle", throttle_cmd)
