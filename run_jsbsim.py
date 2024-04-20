#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

from apscheduler.schedulers.background import BackgroundScheduler   # pip install APScheduler (dnf install python3-APScheduler)
import argparse
from math import cos, sin, tan
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

def easy_fcs():
    control_engine_node.setFloat("throttle", inceptor_node.getFloat("throttle"))
    control_flight_node.setFloat("aileron", inceptor_node.getFloat("aileron"))
    control_flight_node.setFloat("elevator", inceptor_node.getFloat("elevator"))
    control_flight_node.setFloat("elevator_trim", inceptor_node.getFloat("elevator_trim"))
    control_flight_node.setFloat("rudder", inceptor_node.getFloat("rudder"))
    control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
    control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

class NotaPID():
    def __init__(self, name, func, integral_gain, antiwindup, neutral_tolerance):
        self.dt = 0.02
        self.name = name
        self.func = func
        self.int_gain = integral_gain
        self.antiwindup = antiwindup
        self.tol = neutral_tolerance
        self.cmd_neutral = True
        self.cmd_val = 0.0
        self.error_sum = 0.0

    def update(self, rate_cmd, ff_cmd, min_cmd, max_cmd, cur_val, cur_rate):
        if abs(rate_cmd) < self.tol:
            if not self.cmd_neutral:
                # print("set neutral:", self.name)
                self.cmd_val = cur_val
                self.cmd_neutral = True
        else:
            self.cmd_neutral = False
        if self.cmd_neutral:
            ref_rate = (self.cmd_val - cur_val) * 0.05 + ff_cmd
            # print(self.name, ref_rate)
        else:
            ref_rate = rate_cmd + ff_cmd

        if max_cmd is not None and ref_rate > max_cmd:
            ref_rate = max_cmd
        if min_cmd is not None and ref_rate < min_cmd:
            ref_rate = min_cmd

        # print(self.name, ref_rate)
        cmd = self.func(ref_rate)
        self.error_sum += (ref_rate - cur_rate) * self.dt
        if self.error_sum < -self.antiwindup: self.error_sum = -self.antiwindup
        if self.error_sum > self.antiwindup: self.error_sum = self.antiwindup
        cmd += self.error_sum * self.int_gain
        # print(self.name, "ref_val: %.2f" % ref_rate, "error sum: %.2f" % self.error_sum, "%s: %.2f" % (self.name, cmd))
        return cmd

class NotaFCS():
    def __init__(self):
        # filtered state
        self.airspeed_mps = 30
        self.vtrue_mps = 30

        # stick -> rate command scaling
        self.roll_stick_scale = 30 * d2r
        self.pitch_stick_scale = 30 * d2r
        self.yaw_stick_scale = 20

        # envelope protection
        self.alpha_limit_deg = 13.0
        self.bank_limit_deg = 60.0
        self.vne_mps = 80

        # dampers
        self.roll_damp_gain = 1.0
        self.pitch_damp_gain = 1.0
        self.yaw_damp_gain = 5.0

        self.roll_controller = NotaPID("roll", self.roll_func, integral_gain=1.0, antiwindup=0.25, neutral_tolerance=0.02)
        self.pitch_controller = NotaPID("pitch", self.pitch_func, integral_gain=-2.0, antiwindup=0.5, neutral_tolerance=0.03)
        self.yaw_controller = NotaPID("yaw", self.yaw_func, integral_gain=-0.01, antiwindup=10, neutral_tolerance=0.02)

    def update(self):
        airspeed_mps = vel_node.getFloat("vc_mps")
        if airspeed_mps < 30: airspeed_mps = 30
        self.airspeed_mps = 0.99 * self.airspeed_mps + 0.01 * airspeed_mps
        vtrue_mps = vel_node.getFloat("vtrue_mps")
        if vtrue_mps < 30: vtrue_mps = 30
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
        if False:
            # sensed directly (or from sim model)
            self.alpha_deg = aero_node.getFloat("alpha_deg")
            self.beta_deg = aero_node.getFloat("beta_deg")
        else:
            # inertial+airdata estimate (behaves very wrong at low airspeeds, ok in flight!)
            self.alpha_deg = self.alpha_func()
            self.beta_deg = self.beta_func()
        # print("alpha:", self.alpha_deg)
        # print("yaw beta:", beta_deg)

        # presuming a steady state level turn, compute turn rate = func(velocity, bank angle)
        if abs(self.phi_deg) < 89:
            turn_rate_rps = tan(self.phi_deg*d2r) * -gravity / vtrue_mps
        else:
            turn_rate_rps = 0
        # copmute a baseline q and r for the presumed steady state level turn, this is what we dampen towards
        baseline_q = sin(self.phi_deg*d2r) * turn_rate_rps
        baseline_r = cos(self.phi_deg*d2r) * turn_rate_rps

        # print("tr: %.3f" % turn_rate_rps, "q: %.3f %.3f" % (baseline_q, self.q), "r: %.3f %.3f" % (baseline_r, self.r))

        roll_rate_cmd = inceptor_node.getFloat("aileron") * self.roll_stick_scale
        pitch_rate_cmd = -inceptor_node.getFloat("elevator") * self.pitch_stick_scale
        beta_deg_cmd = -inceptor_node.getFloat("rudder") * self.yaw_stick_scale

        # envelope protection (needs to move after or into the controller or at least incorporate the ff term (and dampers?))
        max_q = (self.alpha_limit_deg - self.alpha_deg) * d2r * 2
        # min_q = (self.airspeed_mps - self.vne_mps) * 0.1

        max_p = (self.bank_limit_deg - self.phi_deg) * d2r * 0.5
        min_p = (-self.bank_limit_deg - self.phi_deg) * d2r * 0.5

        # primary flight control laws
        aileron_cmd = self.roll_controller.update(roll_rate_cmd, 0, min_p, max_p, self.phi_deg, self.p)
        elevator_cmd = self.pitch_controller.update(pitch_rate_cmd, baseline_q, None, max_q, self.theta_deg, self.q)
        rudder_cmd = self.yaw_controller.update(beta_deg_cmd, 0, None, None, 0, self.beta_deg)

        # dampers
        aileron_cmd -= self.p * self.roll_damp_gain
        elevator_cmd += (self.q - baseline_q) * self.pitch_damp_gain
        rudder_cmd -= (self.r - baseline_r) * self.yaw_damp_gain

        print("inc_q: %.3f" % (-inceptor_node.getFloat("elevator") * self.pitch_stick_scale), "bl_q: %.3f" % baseline_q, "cmd_q: %.3f" % pitch_rate_cmd,
              "ele: %.3f" % elevator_cmd)

        control_flight_node.setFloat("aileron", aileron_cmd)
        control_flight_node.setFloat("elevator", elevator_cmd)
        control_flight_node.setFloat("rudder", rudder_cmd)

        control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
        control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

        throttle_cmd = inceptor_node.getFloat("throttle")
        control_engine_node.setFloat("throttle", throttle_cmd)

    def alpha_func(self):
        p = 0 # roll rate shows up in our alpha measurement because the alpha vane is at the end of the wing, but let's zero it an ignore that.
        # alpha_deg = -6.519 + 14920.457/self.qbar - 0.331*self.az - 4.432*self.p + 0.243*self.ax + 0.164*self.ay + 3.577*self.q
        alpha_deg = -6.3792 + 14993.7058/self.qbar -0.3121*self.az - 4.3545*p + 5.3980*self.q + 0.2199*self.ax
        return alpha_deg

    def beta_func(self):
        rudder_cmd = inceptor_node.getFloat("rudder")
        throttle_cmd = inceptor_node.getFloat("throttle")
        # beta_deg = 2.807 - 9.752*self.ay + 0.003*self.ay*self.qbar - 5399.632/self.qbar - 0.712*abs(self.ay)
        beta_deg = -0.3552 - 12.1898*rudder_cmd - 3.5411*self.ay + 7.1957*self.r + 0.0008*self.ay*self.qbar + 0.9769*throttle_cmd
        return beta_deg

    def roll_func(self, ref_p):
        roll_cmd = (-73.594 + 2899.711*ref_p + 42.936*self.beta_deg + 3.394*self.airspeed_mps - 80.052*self.ay) / self.qbar
        return roll_cmd

    def pitch_func(self, ref_q):
        throttle_cmd = inceptor_node.getFloat("throttle")
        pitch_cmd = (-292.128 - 1304.394*ref_q - 36.907*abs(self.ay) + 47.111*throttle_cmd - 10.242*self.gbody_z + 2.421*self.az) / self.qbar
        return pitch_cmd

    def yaw_func(self, ref_beta):
        r = 0  # stabilize ... we want the rudder command that gives us ref_beta and r = 0 simultaneously
        # r = vel_node.getFloat("r_rps")
        yaw_cmd = (-51.718 - 103.900*ref_beta + 100.800*self.gbody_x - 1005.478*r - 105.357*self.ay) / self.qbar
        return yaw_cmd

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
