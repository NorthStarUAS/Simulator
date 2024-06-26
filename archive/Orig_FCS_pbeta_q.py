from math import cos, exp, sin, tan
import numpy as np

from lib.constants import d2r, gravity
from lib.props import accel_node, aero_node, att_node, control_engine_node, control_flight_node, inceptor_node, vel_node

class NotaPID():
    def __init__(self, name, min_hold, max_hold, integral_gain, antiwindup, neutral_tolerance):
        self.dt = 0.02
        self.name = name
        self.int_gain = integral_gain
        self.antiwindup = antiwindup
        self.tol = neutral_tolerance
        self.cmd_neutral = True
        self.min_hold = min_hold
        self.max_hold = max_hold
        self.hold_cmd = 0.0
        self.error_sum = 0.0

    def get_ref_value(self, input_cmd, ff_cmd, min_val, max_val, cur_val, flying_confidence):
        if flying_confidence < 0.01:
            self.hold_cmd = cur_val
        if abs(input_cmd) < self.tol:
            if not self.cmd_neutral:
                # print("set neutral:", self.name)
                self.hold_cmd = cur_val
                self.cmd_neutral = True
        else:
            self.cmd_neutral = False
        if self.hold_cmd < self.min_hold:
            self.hold_cmd = self.min_hold
        if self.hold_cmd > self.max_hold:
            self.hold_cmd = self.max_hold
        if self.cmd_neutral:
            error = (self.hold_cmd - cur_val) * flying_confidence
            ref_val = error * 0.05 + ff_cmd
            # print(self.name, ref_rate)
        else:
            ref_val = input_cmd + ff_cmd

        if max_val is not None and ref_val > max_val:
            ref_val = max_val
        if min_val is not None and ref_val < min_val:
            ref_val = min_val
        return ref_val

    def integrator(self, ref_val, cur_val, flying_confidence=0.0):
        cutoff = self.antiwindup * flying_confidence
        self.error_sum += self.int_gain * (ref_val - cur_val) * self.dt
        if self.error_sum < -cutoff: self.error_sum = -cutoff
        if self.error_sum > cutoff: self.error_sum = cutoff
        # print(self.name, "ref_val: %.2f" % ref_val, "error sum: %.2f" % self.error_sum, "%s: %.2f" % (self.name, self.error_sum * self.int_gain))
        return self.error_sum

class Orig_FCS_pbeta_q():
    def __init__(self):
        # filtered state (clamp to minimum of 25 mps because we need to divide
        # by airspeed and qbar so this must be definitely positive 100% of the time.)
        self.airspeed_mps = 25
        self.vtrue_mps = 25

        # stick -> rate command scaling
        self.roll_stick_scale = 30 * d2r
        self.pitch_stick_scale = 30 * d2r
        self.yaw_stick_scale = 20

        # flying vs on ground detection
        self.on_ground_for_sure_mps = 30
        self.flying_for_sure_mps = 40
        self.flying_confidence = 0.0  # range from 0 to 1 representing level of confidence we are on ground(0) vs flying(1)

        # envelope protection
        self.alpha_limit_deg = 13.0
        self.bank_limit_deg = 60.0
        self.vne_mps = 80

        # dampers
        self.roll_damp_gain = 1500.0
        self.pitch_damp_gain = 1500.0
        self.yaw_damp_gain = 6000.0

        self.roll_helper = NotaPID("roll", -45, 45, integral_gain=1.0, antiwindup=0.25, neutral_tolerance=0.02)
        self.pitch_helper = NotaPID("pitch", -15, 15, integral_gain=-4.0, antiwindup=0.5, neutral_tolerance=0.03)
        self.yaw_helper = NotaPID("yaw", -10, 10, integral_gain=-0.01, antiwindup=0.25, neutral_tolerance=0.02)

    def update(self):
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

        # flying?  Let's use a sigmoid function between min/max threshold and
        # compute a 0 - 1 likelihood.
        diff = self.flying_for_sure_mps - self.on_ground_for_sure_mps
        mid = self.on_ground_for_sure_mps + diff * 0.5
        # sigmoid function of [-5 to 5]
        x = 10 * (self.airspeed_mps - self.on_ground_for_sure_mps) / diff - 5
        self.flying_confidence = exp(x) / (1 + exp(x))
        print("flying:", "%.1f %.0f%%" % (self.airspeed_mps, 100*self.flying_confidence))

        if self.flying_confidence > 0.5:
            if True:
                # sensed directly (or from sim model)
                self.alpha_deg = aero_node.getFloat("alpha_deg")
                self.beta_deg = aero_node.getFloat("beta_deg")
            else:
                # inertial+airdata estimate (behaves very wrong at low airspeeds, ok in flight!)
                self.alpha_deg = self.alpha_func()
                self.beta_deg = self.beta_func()  # this functions drifts and can get stuck!
        else:
            self.alpha_deg = self.theta_deg
            self.beta_deg = 0

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

        # Pilot commands
        roll_rate_cmd = inceptor_node.getFloat("aileron") * self.roll_stick_scale
        pitch_rate_cmd = -inceptor_node.getFloat("elevator") * self.pitch_stick_scale
        beta_deg_cmd = -inceptor_node.getFloat("rudder") * self.yaw_stick_scale

        # envelope protection (needs to move after or into the controller or at
        # least incorporate the ff term (and dampers?))  This must consider more
        # than just pitch rate and may need to lower the pitch angle hold value
        # simultaneously, however it takes time for speed to build up and alpha
        # to come down so how/where should the limited 'hold' value get set to?
        max_q = (self.alpha_limit_deg - self.alpha_deg) * d2r * 2
        # min_q = (self.airspeed_mps - self.vne_mps) * 0.1

        # bank angle limits
        max_p = (self.bank_limit_deg - self.phi_deg) * d2r * 0.5
        min_p = (-self.bank_limit_deg - self.phi_deg) * d2r * 0.5

        # Condition and limit the pilot requests
        ref_p = self.roll_helper.get_ref_value(roll_rate_cmd, 0, min_p, max_p, self.phi_deg, self.flying_confidence)
        ref_q = self.pitch_helper.get_ref_value(pitch_rate_cmd, baseline_q, None, max_q, self.theta_deg, self.flying_confidence)
        ref_beta = self.yaw_helper.get_ref_value(beta_deg_cmd, 0, None, None, 0, self.flying_confidence)

        # compute the direct surface position to achieve the command (these
        # functions are fit from the original flight data and involve a matrix
        # inversion that is precomputed and the result is static and never needs
        # to be recomputed.)
        raw_aileron_cmd, raw_rudder_cmd = self.lat_func(ref_p, ref_beta)
        raw_elevator_cmd = self.lon_func(ref_q)

        # run the integrators.  Tip of the hat to imperfect models vs the real
        # world.  The integrators suck up any difference between the model and
        # the real aircraft. Imperfect models can be due to linear fit limits,
        # change in aircraft weight and balance, change in atmospheric
        # conditions, etc.
        aileron_int = self.roll_helper.integrator(ref_p, self.p, self.flying_confidence)
        elevator_int = self.pitch_helper.integrator(ref_q, self.q, self.flying_confidence)
        rudder_int = self.yaw_helper.integrator(ref_beta, self.beta_deg, self.flying_confidence)
        print("integrators: %.2f %.2f %.2f" % (aileron_int, elevator_int, rudder_int))

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        aileron_damp = self.p * self.roll_damp_gain / self.qbar
        elevator_damp = (self.q - baseline_q) * self.pitch_damp_gain / self.qbar
        rudder_damp = (self.r - baseline_r) * self.yaw_damp_gain / self.qbar

        # final output command
        aileron_cmd = raw_aileron_cmd + aileron_int - aileron_damp
        elevator_cmd = raw_elevator_cmd + elevator_int + elevator_damp
        rudder_cmd = raw_rudder_cmd + rudder_int - rudder_damp
        # print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
        #       "raw ele: %.3f" % raw_elevator_cmd, "final ele: %.3f" % elevator_cmd)

        control_flight_node.setFloat("aileron", aileron_cmd)
        control_flight_node.setFloat("elevator", elevator_cmd)
        control_flight_node.setFloat("rudder", rudder_cmd)

        control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
        control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

        throttle_cmd = inceptor_node.getFloat("throttle")
        control_engine_node.setFloat("throttle", throttle_cmd)

    # a simple alpha estimator fit from flight test data
    def alpha_func(self):
        p = 0 # roll rate shows up in our alpha measurement because the alpha vane is at the end of the wing, but let's zero it and ignore that.
        # alpha_deg = -6.519 + 14920.457/self.qbar - 0.331*self.az - 4.432*self.p + 0.243*self.ax + 0.164*self.ay + 3.577*self.q
        alpha_deg = -6.3792 + 14993.7058/self.qbar -0.3121*self.az - 4.3545*p + 5.3980*self.q + 0.2199*self.ax
        return alpha_deg

    # a simple beta estimator fit from flight test data
    def beta_func(self):
        rudder_cmd = inceptor_node.getFloat("rudder")
        # beta_deg = 2.807 - 9.752*self.ay + 0.003*self.ay*self.qbar - 5399.632/self.qbar - 0.712*abs(self.ay)
        beta_deg = -0.3552 - 12.1898*rudder_cmd - 3.5411*self.ay + 7.1957*self.r + 0.0008*self.ay*self.qbar + 0.9769*self.throttle_cmd
        return beta_deg

    # compute model-based aileron and rudder command to simultaneously achieve the reference roll rate and side slip angle.
    Ainv_lat = np.array(
        [[5223.997719570232, 86.53137102359369],
         [3112.870284450966, -187.8833840322353]]
    )
    B_lat = np.array(
        [[-0.3279732547932126, -0.006061380767969274, 0.0017838077680168345, 0.002582130232044947,   8.229002177507066],
         [11.381920691905997,   0.06423929309132188, -0.1514805151401035,   -0.10031783139998209, -318.79044889415076]]
    )
    def lat_func(self, ref_p, ref_beta):
        x = np.array([ref_p, ref_beta])
        b = np.array([1, self.ay, self.gbody_y, self.airspeed_mps, 1/self.airspeed_mps])
        y = (self.Ainv_lat @ x - self.B_lat @ b) / self.qbar
        print("lon y:", y)
        return y.tolist()

    # compute model-based elevator command to achieve the reference pitch rate.
    Ainv_lon = np.array(
        [[-4996.77049111088]]
    )
    B_lon = np.array(
        [[0.15640149796443698, -0.00043212705017340664, 0.01596103011849002, -0.00017520759288595494, -0.0016056595485786098, -5.957540570227146]]
    )
    def lon_func(self, ref_q):
        x = np.array([ref_q])
        b = np.array([1, self.ay, abs(self.ay), self.gbody_y, self.airspeed_mps, 1/self.airspeed_mps])
        y = (self.Ainv_lon @ x - self.B_lon @ b) / self.qbar
        print("lat y:", y)
        return y[0]
