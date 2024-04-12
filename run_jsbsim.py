#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

from apscheduler.schedulers.background import BackgroundScheduler   # pip install APScheduler (dnf install python3-APScheduler)
import argparse
from math import cos, sin
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
pathJSB = os.path.join("/Users/Cirrus/Projects/SVO_Simulator/simulation-python-jsbsim", "JSBSim")
sim = JSBSimWrap(model, pathJSB)
sim.SetupICprops()

if not args.no_trim: # fixme
    trimType = 1  # 1 = in air, 2 = on the ground
    sim.RunTrim(trimType=trimType, throttle=0.5, flap=0.0)
    sim.DispTrim()

def easy_fcs():
    control_engine_node.setFloat("throttle", inceptor_node.getFloat("throttle"))
    control_flight_node.setFloat("aileron", inceptor_node.getFloat("aileron"))
    control_flight_node.setFloat("elevator", inceptor_node.getFloat("elevator"))
    control_flight_node.setFloat("elevator_trim", inceptor_node.getFloat("elevator_trim"))
    control_flight_node.setFloat("rudder", inceptor_node.getFloat("rudder"))
    control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
    control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

class nota_pid():
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

    def update(self, rate_cmd, cur_val, cur_rate):
        if abs(rate_cmd) < self.tol:
            if not self.cmd_neutral:
                print("set neutral:", self.name)
                self.cmd_val = cur_val
                self.cmd_neutral = True
        else:
            self.cmd_neutral = False
        if self.cmd_neutral:
            ref_rate = (self.cmd_val - cur_val) * 0.05
            # print(self.name, ref_rate)
        else:
            ref_rate = rate_cmd
        # print(self.name, ref_rate)
        cmd = self.func(ref_rate)
        self.error_sum += (ref_rate - cur_rate) * self.dt
        if self.error_sum < -self.antiwindup: self.error_sum = -self.antiwindup
        if self.error_sum > self.antiwindup: self.error_sum = self.antiwindup
        cmd += self.error_sum * self.int_gain
        # print(self.name, "ref_val: %.2f" % ref_rate, "error sum: %.2f" % self.error_sum, "%s: %.2f" % (self.name, cmd))
        return cmd

def roll_func(ref_p):
    beta_deg = aero_node.getFloat("beta_deg")
    airspeed_mps = vel_node.getFloat("vc_mps")
    if airspeed_mps < 30: airspeed_mps = 30
    ay = accel_node.getFloat("Ny") * gravity
    rho = 1
    qbar = 0.5 * airspeed_mps**2 * rho

    roll_cmd = (-73.594 + 2899.711*ref_p + 42.936*beta_deg + 3.394*airspeed_mps - 80.052*ay) / qbar

    return roll_cmd

def pitch_func(ref_q):
    ay = accel_node.getFloat("Ny") * gravity
    throttle_cmd = inceptor_node.getFloat("throttle")
    phi_deg = att_node.getFloat("phi_deg")
    theta_deg = att_node.getFloat("theta_deg")
    gbody_z = cos(phi_deg*d2r) * cos(theta_deg*d2r) * gravity
    az = accel_node.getFloat("Nz") * gravity
    airspeed_mps = vel_node.getFloat("vc_mps")
    if airspeed_mps < 30: airspeed_mps = 30
    rho = 1
    qbar = 0.5 * airspeed_mps**2 * rho

    pitch_cmd = (-292.128 - 1304.394*ref_q - 36.907*abs(ay) + 47.111*throttle_cmd - 10.242*gbody_z + 2.421*az) / qbar

    return pitch_cmd

def yaw_func(ref_beta):
    theta_deg = att_node.getFloat("theta_deg")
    gbody_x = -sin(theta_deg*d2r) * gravity
    ay = accel_node.getFloat("Ny") * gravity
    airspeed_mps = vel_node.getFloat("vc_mps")
    if airspeed_mps < 30: airspeed_mps = 30
    rho = 1
    qbar = 0.5 * airspeed_mps**2 * rho

    r = 0  # stabilize ... we want the rudder command that gives us ref_beta and r = 0 simultaneously
    # r = vel_node.getFloat("r_rps")
    yaw_cmd = (-51.718 - 103.900*ref_beta + 100.800*gbody_x - 1005.478*r - 105.357*ay) / qbar

    return yaw_cmd

# stick -> rate command scaling
roll_stick_scale = 30 * d2r
pitch_stick_scale = 20 * d2r
yaw_stick_scale = 20

# envelope protection
alpha_limit_deg = 12.0

# dampers
roll_damp_gain = 0.4
pitch_damp_gain = 0.6
yaw_damp_gain = 0.5

roll_controller = nota_pid("roll", roll_func, integral_gain=1.0, antiwindup=0.25, neutral_tolerance=0.02)
pitch_controller = nota_pid("pitch", pitch_func, integral_gain=-1.0, antiwindup=0.5, neutral_tolerance=0.03)
yaw_controller = nota_pid("yaw", yaw_func, integral_gain=-0.01, antiwindup=10, neutral_tolerance=0.02)

def nota_fcs():
    alpha_deg = aero_node.getFloat("alpha_deg")
    beta_deg = aero_node.getFloat("beta_deg")
    print("alpha:", alpha_deg)
    # print("yaw beta:", beta_deg)
    phi_deg = att_node.getFloat("phi_deg")
    theta_deg = att_node.getFloat("theta_deg")
    p = vel_node.getFloat("p_rps")
    q = vel_node.getFloat("q_rps")
    r = vel_node.getFloat("r_rps")

    roll_cmd = inceptor_node.getFloat("aileron") * roll_stick_scale
    pitch_cmd = -inceptor_node.getFloat("elevator") * pitch_stick_scale
    yaw_cmd = -inceptor_node.getFloat("rudder") * yaw_stick_scale

    # envelope protection
    max_q = (alpha_limit_deg - alpha_deg) * d2r * 1
    if pitch_cmd > max_q:
        pitch_cmd = max_q

    # primary flight control laws
    aileron_cmd = roll_controller.update(roll_cmd, phi_deg, p)
    elevator_cmd = pitch_controller.update(pitch_cmd, theta_deg, q)
    rudder_cmd = yaw_controller.update(yaw_cmd, 0, beta_deg)

    # dampers
    aileron_cmd -= p * roll_damp_gain
    elevator_cmd += q * pitch_damp_gain
    rudder_cmd -= r * yaw_damp_gain

    control_flight_node.setFloat("aileron", aileron_cmd)
    control_flight_node.setFloat("elevator", elevator_cmd)
    control_flight_node.setFloat("rudder", rudder_cmd)

    control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
    control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

    throttle_cmd = inceptor_node.getFloat("throttle")
    control_engine_node.setFloat("throttle", throttle_cmd)

def update():
    joystick.update()
    nota_fcs()
    sim.RunSteps(4, updateWind=True)
    sim.PublishProps()

    # fgfs.send_to_fgfs()
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
