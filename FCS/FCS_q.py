from math import cos, sin, tan
import numpy as np

from lib.constants import d2r, gravity
from lib.props import accel_node, aero_node, att_node, control_flight_node, inceptor_node, vel_node

from .NotaPID import NotaPID

class FCS_q():
    def __init__(self):
        # filtered state (clamp to minimum of 25 mps because we need to divide
        # by airspeed and qbar so this must be definitely positive 100% of the time.)
        self.airspeed_mps = 25
        self.vtrue_mps = 25

        # stick -> rate command scaling
        self.pitch_stick_scale = 30 * d2r

        # envelope protection
        self.alpha_limit_deg = 13.0
        self.vne_mps = 80

        # integrators
        self.elevator_int = 0.0

        # dampers
        self.pitch_damp_gain = 1500.0

        self.pitch_helper = NotaPID("pitch", -15, 15, integral_gain=-4.0, antiwindup=0.5, neutral_tolerance=0.03)

    def update(self, flying_confidence):
        # fetch and compute all the values needed by the control laws
        self.throttle_cmd = inceptor_node.getFloat("throttle")

        airspeed_mps = vel_node.getFloat("vc_mps")
        if airspeed_mps < 25: airspeed_mps = 25
        self.airspeed_mps = 0.99 * self.airspeed_mps + 0.01 * airspeed_mps
        vtrue_mps = vel_node.getFloat("vtrue_mps")
        if vtrue_mps < 25: vtrue_mps = 25
        self.vtrue_mps = 0.99 * self.vtrue_mps + 0.01 * vtrue_mps
        rho = 1.225
        self.qbar = 0.5 * self.airspeed_mps**2 * rho

        self.phi_deg = att_node.getFloat("phi_deg")
        self.theta_deg = att_node.getFloat("theta_deg")
        self.p = vel_node.getFloat("p_rps")
        self.q = vel_node.getFloat("q_rps")
        self.r = vel_node.getFloat("r_rps")
        self.ax = accel_node.getFloat("Nx") * gravity
        self.ay = accel_node.getFloat("Ny") * gravity
        self.az = accel_node.getFloat("Nz") * gravity
        self.gbody_x = -sin(self.theta_deg*d2r) * gravity
        self.gbody_y = sin(self.phi_deg*d2r) * cos(self.theta_deg*d2r) * gravity
        self.gbody_z = cos(self.phi_deg*d2r) * cos(self.theta_deg*d2r) * gravity
        self.q_term1 = sin(self.phi_deg*d2r) * (sin(self.phi_deg*d2r) / cos(self.phi_deg*d2r)) / self.airspeed_mps

        if flying_confidence > 0.5:
            if True:
                # sensed directly (or from sim model)
                self.alpha_deg = aero_node.getFloat("alpha_deg")
            else:
                # inertial+airdata estimate (behaves very wrong at low airspeeds, ok in flight!)
                self.alpha_deg = self.alpha_func()
        else:
            self.alpha_deg = self.theta_deg

        # Feed forward steady state q and r basd on bank angle/turn rate.
        # Presuming a steady state level turn, compute turn rate =
        # func(velocity, bank angle).  This is the one feed forward term used in
        # this set of control laws and it is purely physics based and works for
        # all fixed wing aircraft.
        if abs(self.phi_deg) < 89:
            turn_rate_rps = tan(self.phi_deg*d2r) * -gravity / vtrue_mps
        else:
            turn_rate_rps = 0
        # compute a baseline q and r for the presumed steady state level turn,
        # this is what we dampen towards
        baseline_q = sin(self.phi_deg*d2r) * turn_rate_rps
        baseline_r = cos(self.phi_deg*d2r) * turn_rate_rps
        # print("tr: %.3f" % turn_rate_rps, "q: %.3f %.3f" % (baseline_q, self.q), "r: %.3f %.3f" % (baseline_r, self.r))

        # Pilot command
        pitch_rate_cmd = -inceptor_node.getFloat("elevator") * self.pitch_stick_scale

        # envelope protection (needs to move after or into the controller or at
        # least incorporate the ff term (and dampers?))  This must consider more
        # than just pitch rate and may need to lower the pitch angle hold value
        # simultaneously, however it takes time for speed to build up and alpha
        # to come down so how/where should the limited 'hold' value get set to?
        max_q = (self.alpha_limit_deg - self.alpha_deg) * d2r * 2
        # min_q = (self.airspeed_mps - self.vne_mps) * 0.1

        # Condition and limit the pilot request
        ref_q = self.pitch_helper.get_ref_value(pitch_rate_cmd, baseline_q, None, max_q, self.theta_deg, flying_confidence)

        # compute the direct surface position to achieve the command (these
        # functions are fit from the original flight data and involve a matrix
        # inversion that is precomputed and the result is static and never needs
        # to be recomputed.)
        raw_elevator_cmd = self.lon_func(ref_q)

        # run the integrators.  Tip of the hat to imperfect models vs the real
        # world.  The integrators suck up any difference between the model and
        # the real aircraft. Imperfect models can be due to linear fit limits,
        # change in aircraft weight and balance, change in atmospheric
        # conditions, etc.
        self.elevator_int = self.pitch_helper.integrator(ref_q, self.q, flying_confidence)
        # print("pitch integrators: %.2f %.2f %.2f" % (aileron_int, self.elevator_int, rudder_int))  # move outside

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        elevator_damp = (self.q - baseline_q) * self.pitch_damp_gain / self.qbar

        # final output command
        elevator_cmd = raw_elevator_cmd + self.elevator_int + elevator_damp
        # print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
        #       "raw ele: %.3f" % raw_elevator_cmd, "final ele: %.3f" % elevator_cmd)
        control_flight_node.setFloat("elevator", elevator_cmd)

    # a simple alpha estimator fit from flight test data
    def alpha_func(self):
        p = 0 # roll rate shows up in our alpha measurement because the alpha vane is at the end of the wing, but let's zero it and ignore that.
        # alpha_deg = -6.519 + 14920.457/self.qbar - 0.331*self.az - 4.432*self.p + 0.243*self.ax + 0.164*self.ay + 3.577*self.q
        alpha_deg = -6.3792 + 14993.7058/self.qbar -0.3121*self.az - 4.3545*p + 5.3980*self.q + 0.2199*self.ax
        return alpha_deg

    # compute model-based elevator command to achieve the reference pitch rate.
    def lon_func(self, ref_q):
        Ainv = np.array(
            [[-4996.77049111088]]
        )
        B = np.array(
            [[0.15640149796443698, -0.00043212705017340664, 0.01596103011849002, -0.00017520759288595494, -0.0016056595485786098, -5.957540570227146]]
        )
        x = np.array([ref_q])
        b = np.array([1, self.ay, abs(self.ay), self.gbody_y, self.airspeed_mps, 1/self.airspeed_mps])
        y = (Ainv @ x - B @ b) / self.qbar
        print("lat y:", y)
        return y[0]
