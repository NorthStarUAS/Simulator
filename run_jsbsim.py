#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

from apscheduler.schedulers.background import BackgroundScheduler   # pip install APScheduler (dnf install python3-APScheduler)
import argparse
from math import cos, sin, tan
import numpy as np
import os
import time

from lib.constants import d2r, gravity
from lib.jsbsim import JSBSimWrap
from lib.joystick import Joystick
from lib.props import accel_node, aero_node, att_node, control_engine_node, control_flight_node, inceptor_node, vel_node
from visuals.fgfs import fgfs
from visuals.pvi.pvi import PVI
from visuals.xp.xp import XPlane

# command line arguments
parser = argparse.ArgumentParser(description="run the simulation")
parser.add_argument("model", help="flight model")
parser.add_argument('--realtime', action='store_true', help='run sim in realtime')
parser.add_argument('--no-trim', action='store_true', help="don't trim")
args = parser.parse_args()

run_time = 600

joystick = Joystick()
pvi = PVI()
xp = XPlane()

model = 'SR22T'
pathJSB = os.path.join("/home/clolson/Projects/SVO_Simulator/simulation-python-jsbsim", "JSBSim")
# pathJSB = os.path.join("/Users/Cirrus/Projects/SVO_Simulator/simulation-python-jsbsim", "JSBSim")
sim = JSBSimWrap(model, pathJSB)
sim.SetupICprops()

if not args.no_trim: # fixme
    trimType = 1  # 1 = in air, 2 = on the ground
    sim.RunTrim(trimType=trimType, throttle=0.5, flap=0.0)
    sim.DispTrim()
# sim.SetTurb(turbSeverity=1, vWind20_mps=2.5, vWindHeading_deg=270) # Trim with wind, no turbulence

def direct_fcs():
    control_engine_node.setFloat("throttle", inceptor_node.getFloat("throttle"))
    control_flight_node.setFloat("aileron", inceptor_node.getFloat("aileron"))
    control_flight_node.setFloat("elevator", inceptor_node.getFloat("elevator"))
    control_flight_node.setFloat("elevator_trim", inceptor_node.getFloat("elevator_trim"))
    control_flight_node.setFloat("rudder", inceptor_node.getFloat("rudder"))
    control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
    control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

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

    def get_ref_value(self, input_cmd, ff_cmd, min_val, max_val, cur_val, is_flying):
        if not is_flying:
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
            ref_val = (self.hold_cmd - cur_val) * 0.05 + ff_cmd
            # print(self.name, ref_rate)
        else:
            ref_val = input_cmd + ff_cmd

        if max_val is not None and ref_val > max_val:
            ref_val = max_val
        if min_val is not None and ref_val < min_val:
            ref_val = min_val
        return ref_val

    def integrator(self, ref_val, cur_val, is_flying=False):
        if is_flying:
            self.error_sum += self.int_gain * (ref_val - cur_val) * self.dt
            if self.error_sum < -self.antiwindup: self.error_sum = -self.antiwindup
            if self.error_sum > self.antiwindup: self.error_sum = self.antiwindup
            # print(self.name, "ref_val: %.2f" % ref_val, "error sum: %.2f" % self.error_sum, "%s: %.2f" % (self.name, self.error_sum * self.int_gain))
        else:
            fade_sec = 2
            self.error_sum *= (fade_sec - self.dt) / fade_sec
        return self.error_sum

# Issue 1: life is good through take off roll and rotation, but when speed
# threshold trips and integrators turn on, that stick back pressure for holding
# the nose up turns into a real rate command and the nose pitches up fast and
# pilot has to push back and then center the stick.

class NotaFCS():
    def __init__(self):
        # filtered state (clamp to minimum of 25 mps because we need to divide
        # by airspeed and qbar so this must be definitely positive.)
        self.airspeed_mps = 25
        self.vtrue_mps = 25

        # stick -> rate command scaling
        self.roll_stick_scale = 30 * d2r
        self.pitch_stick_scale = 30 * d2r
        self.yaw_stick_scale = 20

        # flying vs on ground detection
        self.on_ground_for_sure_mps = 30
        self.flying_for_sure_mps = 40
        self.is_flying = False

        # envelope protection
        self.alpha_limit_deg = 13.0
        self.bank_limit_deg = 60.0
        self.vne_mps = 80

        # dampers
        self.roll_damp_gain = 1.0
        self.pitch_damp_gain = 1.0
        self.yaw_damp_gain = 5.0

        self.roll_helper = NotaPID("roll", -45, 45, integral_gain=1.0, antiwindup=0.25, neutral_tolerance=0.02)
        self.pitch_helper = NotaPID("pitch", -15, 15, integral_gain=-4.0, antiwindup=0.5, neutral_tolerance=0.03)
        self.yaw_helper = NotaPID("yaw", -10, 10, integral_gain=-0.01, antiwindup=0.25, neutral_tolerance=0.02)

        self.Ainv_lat = np.array(
            [[5775.381885098956, 82.77808579660751],
             [3174.049921950911, -208.26398785983227]]
        )
        self.B_lat = np.array(
            [[-0.005241737213195004, -0.005865478300975062, 0.0022233286464520595, -1.4980540274253145],
             [-1.1302438460613164, 0.059599793907393415, -0.17246330454816164, 62.39945825500266]]
        )
        self.Ainv_lon = np.array(
            [[-2916.4286901639866]]
        )
        self.B_lon = np.array(
            [[-0.022590773981529783, -2.4757330979504637, 1.2706001552602082]]
        )

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

        # flying?
        if self.airspeed_mps < self.on_ground_for_sure_mps:
            self.is_flying = False
        if self.airspeed_mps > self.flying_for_sure_mps:
            self.is_flying = True
        print("flying:", "%.1f" % self.airspeed_mps, self.is_flying)

        if self.is_flying:
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

        if self.is_flying:
            # Feed forward steady state q and r basd on bank angle/turn rate.
            # Presuming a steady state level turn, compute turn rate =
            # func(velocity, bank angle).  This is the one feed forward term used in
            # this set of control laws and it is purely physics based and
            # works for all fixed wing aircraft.
            if abs(self.phi_deg) < 89:
                turn_rate_rps = tan(self.phi_deg*d2r) * -gravity / vtrue_mps
            else:
                turn_rate_rps = 0
            # compute a baseline q and r for the presumed steady state level turn, this is what we dampen towards
            baseline_q = sin(self.phi_deg*d2r) * turn_rate_rps
            baseline_r = cos(self.phi_deg*d2r) * turn_rate_rps
            # print("tr: %.3f" % turn_rate_rps, "q: %.3f %.3f" % (baseline_q, self.q), "r: %.3f %.3f" % (baseline_r, self.r))
        else:
            baseline_q = 0
            baseline_r = 0

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
        ref_p = self.roll_helper.get_ref_value(roll_rate_cmd, 0, min_p, max_p, self.phi_deg, self.is_flying)
        ref_q = self.pitch_helper.get_ref_value(pitch_rate_cmd, baseline_q, None, max_q, self.theta_deg, self.is_flying)
        ref_beta = self.yaw_helper.get_ref_value(beta_deg_cmd, 0, None, None, 0, self.is_flying)

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
        aileron_int = self.roll_helper.integrator(ref_p, self.p, self.is_flying)
        elevator_int = self.pitch_helper.integrator(ref_q, self.q, self.is_flying)
        rudder_int = self.yaw_helper.integrator(ref_beta, self.beta_deg, self.is_flying)
        print("integrators: %.2f %.2f %.2f" % (aileron_int, elevator_int, rudder_int))

        # dampers, these can be tuned to pilot preference for lighter finger tip
        # flying vs heavy stable flying.
        aileron_damp = self.p * self.roll_damp_gain
        elevator_damp = (self.q - baseline_q) * self.pitch_damp_gain
        rudder_damp = (self.r - baseline_r) * self.yaw_damp_gain

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
    def lat_func(self, ref_p, ref_beta):
        x = np.array([ref_p, ref_beta])
        b = np.array([1, self.ay, self.gbody_y, 1/self.airspeed_mps])
        y = (self.Ainv_lat @ x - self.B_lat @ b) / self.qbar
        print("lon y:", y)
        return y.tolist()

    # compute model-based elevator command to achieve the reference pitch rate.
    def lon_func(self, ref_q):
        x = np.array([ref_q])
        b = np.array([1, 1/self.airspeed_mps, self.q_term1])
        y = (self.Ainv_lon @ x - self.B_lon @ b) / self.qbar
        print("lat y:", y)
        return y[0]

fcs = NotaFCS()

def update():
    joystick.update()
    fcs.update()
    sim.RunSteps(4, updateWind=True)
    sim.PublishProps()

    fgfs.send_to_fgfs()
    # pvi.update(state_mgr, 0, 0, 0, 0)
    xp.update()

if args.realtime:
    sched = BackgroundScheduler()
    sched.add_job(update, 'interval', seconds=sim.dt*4)
    sched.start()
    while True:
        time.sleep(run_time)
    sched.shutdown()
else:
    while sim.time <= run_time:
        sim.update()

sim.plot()
