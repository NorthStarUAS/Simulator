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
    def __init__(self, name, func, integral_gain, antiwindup, neutral_tolerance):
        self.dt = 0.02
        self.name = name
        self.func = func
        self.int_gain = integral_gain
        self.antiwindup = antiwindup
        self.tol = neutral_tolerance
        self.cmd_neutral = True
        self.hold_cmd = 0.0
        self.error_sum = 0.0

    def get_ref_value(self, input_cmd, ff_cmd, min_val, max_val, hold_val):
        if abs(input_cmd) < self.tol:
            if not self.cmd_neutral:
                # print("set neutral:", self.name)
                self.hold_cmd = hold_val
                self.cmd_neutral = True
        else:
            self.cmd_neutral = False
        if self.cmd_neutral:
            ref_val = (self.hold_cmd - hold_val) * 0.05 + ff_cmd
            # print(self.name, ref_rate)
        else:
            ref_val = input_cmd + ff_cmd

        if max_val is not None and ref_val > max_val:
            ref_val = max_val
        if min_val is not None and ref_val < min_val:
            ref_val = min_val
        return ref_val

    def integrator(self, ref_val, cur_val):
        self.error_sum += (ref_val - cur_val) * self.dt
        if self.error_sum < -self.antiwindup: self.error_sum = -self.antiwindup
        if self.error_sum > self.antiwindup: self.error_sum = self.antiwindup
        # print(self.name, "ref_val: %.2f" % ref_val, "error sum: %.2f" % self.error_sum, "%s: %.2f" % (self.name, self.error_sum * self.int_gain))
        return self.error_sum * self.int_gain

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
        self.pitch_controller = NotaPID("pitch", self.pitch_func, integral_gain=-4.0, antiwindup=0.5, neutral_tolerance=0.03)
        self.yaw_controller = NotaPID("yaw", self.yaw_func, integral_gain=-0.01, antiwindup=10, neutral_tolerance=0.02)

        self.A_1 =  np.array(
            [[  640.5491204237019, 7318.274338542231,  145.36316444343038],
             [-5925.7563775981625, 8166.787947667861,   71.97155383929868],
             [ -525.2920046924208, 2808.335317363771, -137.2025832944612]]
        )

        self.B = np.array(
            [[-0.05693361893060279, -0.001295528867971077, -0.005711799151942839, 0.000905503147267076, -0.009470654619625466,  0.002075155854446521,  0.017540533979970362,  0.0017961339985038977,  4.634338692782735, 0.0016511736062629167, -10.000622489234617],
             [-0.2618144176877694,   0.0025267035433311023, 0.008192753236485892, 0.004585894999890392,  0.023859996183603096, -0.0019562397939452947, 0.004690599025956665, -0.0027451715297843546,  7.65157976906464,  0.0033548602796592184,  11.862452693894255],
             [ 1.267147891398938,   -0.06615263213349391,  -0.040798310823740226, 0.005928758316797655,  0.5826256649631434,   -0.1468643099012692,   -0.8535965934839992,   -0.04164329779085257, -286.101582481374,   -0.08068076777562076,   326.9646738668046]]
        )

    def update(self):
        self.throttle_cmd = inceptor_node.getFloat("throttle")

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
        self.q_term1 = sin(self.phi_deg*d2r) * (sin(self.phi_deg*d2r) / cos(self.phi_deg*d2r)) / self.airspeed_mps

        if True:
            # sensed directly (or from sim model)
            self.alpha_deg = aero_node.getFloat("alpha_deg")
            self.beta_deg = aero_node.getFloat("beta_deg")
        else:
            # inertial+airdata estimate (behaves very wrong at low airspeeds, ok in flight!)
            self.alpha_deg = self.alpha_func()
            self.beta_deg = self.beta_func()

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
        # aileron_cmd = self.roll_controller.update(roll_rate_cmd, 0, min_p, max_p, self.phi_deg, self.p)
        # elevator_cmd = self.pitch_controller.update(pitch_rate_cmd, baseline_q, None, max_q, self.theta_deg, self.q)
        # rudder_cmd = self.yaw_controller.update(beta_deg_cmd, 0, None, None, 0, self.beta_deg)
        ref_p = self.roll_controller.get_ref_value(roll_rate_cmd, 0, min_p, max_p, self.phi_deg)
        ref_q = self.pitch_controller.get_ref_value(pitch_rate_cmd, baseline_q, None, max_q, self.theta_deg)
        ref_beta = self.yaw_controller.get_ref_value(beta_deg_cmd, 0, None, None, 0)

        # direct surface position control
        # raw_aileron_cmd = self.roll_func(ref_p, ref_q, ref_beta)
        # raw_elevator_cmd = self.pitch_func(ref_p, ref_q, ref_beta)
        # raw_rudder_cmd = self.yaw_func(ref_p, ref_q, ref_beta)
        raw_aileron_cmd, raw_elevator_cmd, raw_rudder_cmd = self.roll_it_all_together_func(ref_p, ref_q, ref_beta)

        # integrators
        aileron_int = self.roll_controller.integrator(ref_p, self.p)
        elevator_int = self.pitch_controller.integrator(ref_q, self.q)
        rudder_int = self.yaw_controller.integrator(ref_beta, self.beta_deg)
        print("integrators: %.2f %.2f %.2f" % (aileron_int, elevator_int, rudder_int))

        # dampers
        aileron_damp = self.p * self.roll_damp_gain
        elevator_damp = (self.q - baseline_q) * self.pitch_damp_gain
        rudder_damp = (self.r - baseline_r) * self.yaw_damp_gain

        # final output command
        aileron_cmd = raw_aileron_cmd + aileron_int - aileron_damp
        elevator_cmd = raw_elevator_cmd + elevator_int + elevator_damp
        rudder_cmd = raw_rudder_cmd + rudder_int - rudder_damp

        print("inc_q: %.3f" % pitch_rate_cmd, "bl_q: %.3f" % baseline_q, "ref_q: %.3f" % ref_q,
              "raw ele: %.3f" % raw_elevator_cmd, "final ele: %.3f" % elevator_cmd)

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
        # beta_deg = 2.807 - 9.752*self.ay + 0.003*self.ay*self.qbar - 5399.632/self.qbar - 0.712*abs(self.ay)
        beta_deg = -0.3552 - 12.1898*rudder_cmd - 3.5411*self.ay + 7.1957*self.r + 0.0008*self.ay*self.qbar + 0.9769*self.throttle_cmd
        return beta_deg

    def roll_func(self, ref_p, ref_q, ref_beta):
        # roll_cmd = (-73.594 + 2899.711*ref_p + 42.936*self.beta_deg + 3.394*self.airspeed_mps - 80.052*self.ay) / self.qbar
        roll_cmd = (328.3374 - 24.4623*self.alpha_deg + 73.0825*ref_beta + 1549.7883*ref_p + 289.0764*ref_q + 594.0779*self.r + 43.4224*self.gbody_x + 14.4835*self.gbody_y + 14.1115*self.gbody_z - 6.7697*abs(self.gbody_y) + 3.4586*self.ax + 3.8059*self.ay - 3.0352*self.az + 1.9924*abs(self.ay) - 94.2333*self.throttle_cmd) / self.qbar
        return roll_cmd

    def pitch_func(self, ref_p, ref_q, ref_beta):
        # pitch_cmd = (-292.128 - 1304.394*ref_q - 36.907*abs(self.ay) + 47.111*self.throttle_cmd - 10.242*self.gbody_z + 2.421*self.az) / self.qbar
        pitch_cmd = (-510.0228 - 15.2499*self.alpha_deg - 3.8565*ref_beta - 30.3212*ref_p - 5.4334*ref_q + 77.0022*self.r + 2.1043*self.gbody_x + 0.0880*self.gbody_y - 31.3148*self.gbody_z - 7.4353*abs(self.gbody_y) - 2.3501*self.ax - 4.0020*self.ay + 0.6556*self.az - 0.2947*abs(self.ay) + 60.1885*self.throttle_cmd) / self.qbar
        return pitch_cmd

    def yaw_func(self, ref_p, ref_q, ref_beta):
        r = 0  # stabilize ... we want the rudder command that gives us ref_beta and r = 0 simultaneously
        # r = vel_node.getFloat("r_rps")
        # yaw_cmd = (-51.718 - 103.900*ref_beta + 100.800*self.gbody_x - 1005.478*r - 105.357*self.ay) / self.qbar
        yaw_cmd = (-495.2686 + 27.9346*self.alpha_deg - 98.6537*ref_beta - 1.5749*ref_p - 72.1812*ref_q + 1995.3476*r + 92.9930*self.gbody_x + 15.1753*self.gbody_y - 28.5866*self.gbody_z + 12.1165*abs(self.gbody_y) - 7.5128*self.ax - 6.4226*self.ay + 1.6634*self.az - 2.9201*abs(self.ay) + 173.3109*self.throttle_cmd) / self.qbar
        return yaw_cmd

    def roll_it_all_together_func(self, ref_p, ref_q, ref_beta):
        x = np.array([ref_p, ref_q, ref_beta])
        b = np.array([1, self.ax, self.ay, self.az, self.gbody_x, self.gbody_y, self.gbody_z, abs(self.gbody_y), 1/self.airspeed_mps, self.airspeed_mps, self.q_term1])
        y = (self.A_1 * x - self.B * b) / self.qbar
        print("y:", y)
        return y.tolist()

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
