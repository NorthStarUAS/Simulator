from math import cos, pi, sin, tan
from lib.constants import d2r, gravity
from lib.props import aero_node, att_node, control_engine_node, control_flight_node, fcs_node, inceptor_node, vel_node

from FCS.direct_airdata import alpha_func, beta_func
from FCS.direct_pbeta import pbeta_controller
from FCS.direct_q import q_controller
from FCS.util import IsFlying

rho = 1.225

class FCSMgr():
    def __init__(self):
        # stick -> rate command scaling
        self.roll_stick_scale = 30 * d2r   # rad
        self.pitch_stick_scale = 20 * d2r  # rad
        self.yaw_stick_scale = 20          # maps to beta_deg

        self.fcs_lat = pbeta_controller()
        self.fcs_lon = q_controller()
        self.is_flying = IsFlying(on_ground_for_sure_mps=30, flying_for_sure_mps=40)

        # filtered state (clamp to minimum of 25 mps because we need to divide
        # by airspeed and qbar so this must be definitely positive 100% of the time.)
        self.vc_filt_mps = 25
        self.vtrue_filt_mps = 25

    def compute_stuff(self):
        # update state and filters

        # velocity terms
        vc_mps = vel_node.getDouble("vc_mps")
        if vc_mps < 25: vc_mps = 25
        self.vc_filt_mps = 0.99 * self.vc_filt_mps + 0.01 * vc_mps
        vtrue_mps = vel_node.getDouble("vtrue_mps")
        if vtrue_mps < 25: vtrue_mps = 25
        self.vtrue_filt_mps = 0.99 * self.vtrue_filt_mps + 0.01 * vtrue_mps
        qbar = 0.5 * self.vc_filt_mps**2 * rho
        fcs_node.setDouble("vc_filt_mps", self.vc_filt_mps)
        fcs_node.setDouble("qbar", qbar)

        # in the air vs on the ground?  (uses a sigmoid function between
        # threshold speeds)
        flying_confidence = self.is_flying.get_flying_confidence(self.vc_filt_mps)
        fcs_node.setDouble("flying_confidence", flying_confidence)

        # alpha / beta estimates (or direct from sim model)
        if flying_confidence > 0.5:
            if True:
                # sensed directly (or from sim model)
                alpha_deg = aero_node.getDouble("alpha_deg")
                beta_deg = aero_node.getDouble("beta_deg")
            else:
                # inertial+airdata estimate (behaves very wrong at low airspeeds, ok in flight!)
                rudder_cmd = inceptor_node.getDouble("yaw")
                throttle_cmd = inceptor_node.getDouble("power")
                alpha_deg = alpha_func(qbar, az, p, q, ax)
                beta_deg = beta_func(qbar, ay, r, rudder_cmd, throttle_cmd)  # this functions drifts and can get stuck!
        else:
            alpha_deg = att_node.getDouble("theta_deg")
            beta_deg = 0
        fcs_node.setDouble("alpha_deg", alpha_deg)
        fcs_node.setDouble("beta_deg", beta_deg)
        print("beta: %.1f" % beta_deg)

        # Feed forward steady state q and r basd on bank angle/turn rate.
        # Presuming a steady state level turn, compute turn rate =
        # func(velocity, bank angle).  This is the one feed forward term used in
        # this set of control laws and it is purely physics based and works for
        # all fixed wing aircraft.
        phi_rad = att_node.getDouble("phi_deg") * d2r
        if abs(phi_rad) < pi * 0.5 * 0.9:
            turn_rate_rps = tan(phi_rad) * -gravity / self.vtrue_filt_mps
        else:
            turn_rate_rps = 0
        # compute a baseline q and r for the presumed steady state level turn,
        # this is what we dampen towards
        baseline_q = sin(phi_rad) * turn_rate_rps
        baseline_r = cos(phi_rad) * turn_rate_rps
        # print("tr: %.3f" % turn_rate_rps, "q: %.3f %.3f" % (baseline_q, self.q), "r: %.3f %.3f" % (baseline_r, self.r))
        fcs_node.setDouble("baseline_q", baseline_q)
        fcs_node.setDouble("baseline_r", baseline_r)

    def update(self):
        # update state and filters
        self.compute_stuff()

        # pilot commands
        roll_rate_request = inceptor_node.getDouble("roll") * self.roll_stick_scale
        pitch_rate_request = -inceptor_node.getDouble("pitch") * self.pitch_stick_scale
        beta_deg_request = -inceptor_node.getDouble("yaw") * self.yaw_stick_scale

        # flight control laws
        roll_cmd, yaw_cmd = self.fcs_lat.update(roll_rate_request, beta_deg_request)
        pitch_cmd = self.fcs_lon.update(pitch_rate_request)
        print("integrators: %.2f %.2f %.2f" % (self.fcs_lat.roll_int, self.fcs_lon.pitch_int, self.fcs_lat.yaw_int))
        control_flight_node.setDouble("aileron", roll_cmd)
        control_flight_node.setDouble("rudder", yaw_cmd)
        control_flight_node.setDouble("elevator", pitch_cmd)

        # pass through flaps and throttle for now
        control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
        control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))
        throttle_cmd = inceptor_node.getDouble("power")
        control_engine_node.setDouble("throttle", throttle_cmd)
