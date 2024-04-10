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
pathJSB = os.path.join("/home/clolson/Projects/SVO_Simulator/simulation-python-jsbsim", "JSBSim")
sim = JSBSimWrap(model, pathJSB)
sim.SetupICprops()

roll_cmd_neutral = True
ref_phi = 0.0
sum_p = 0.0

pitch_cmd_neutral = True
ref_theta = 0.0
sum_q = 0.0

yaw_cmd_neutral = True
ref_beta = 0.0
sum_beta = 0.0

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

def nota_fcs():
    global roll_cmd_neutral
    global ref_phi
    global sum_p

    global pitch_cmd_neutral
    global ref_theta
    global sum_q

    global yaw_cmd_neutral
    global ref_beta
    global sum_beta

    rho = 1
    beta_deg = aero_node.getFloat("beta_deg")
    ay = accel_node.getFloat("Ny") * gravity
    az = accel_node.getFloat("Nz") * gravity
    airspeed_mps = vel_node.getFloat("vc_mps")
    if airspeed_mps < 30: airspeed_mps = 30
    qbar = 0.5 * airspeed_mps**2 * rho
    throttle_cmd = inceptor_node.getFloat("throttle")
    psi_deg = att_node.getFloat("phi_deg")
    theta_deg = att_node.getFloat("theta_deg")
    phi_deg = att_node.getFloat("phi_deg")
    gbody_x = -sin(theta_deg*d2r) * gravity
    gbody_z = cos(psi_deg*d2r) * cos(theta_deg*d2r) * gravity
    control_engine_node.setFloat("throttle", throttle_cmd)
    p = vel_node.getFloat("p_rps")
    q = vel_node.getFloat("q_rps")
    r = vel_node.getFloat("r_rps")

    if abs(inceptor_node.getFloat("aileron")) < 0.01:
        if not roll_cmd_neutral:
            print("set roll neutral")
            ref_phi = phi_deg
            roll_cmd_neutral = True
    else:
        roll_cmd_neutral = False
    if roll_cmd_neutral:
        ref_p = (ref_phi - phi_deg) * 0.05
    else:
        ref_p = inceptor_node.getFloat("aileron") * 20 * d2r
    # aileron = 0.089 + 1.216*ref_p
    # aileron = 0.061 + 1.275*ref_p + 0.031*beta_deg
    # aileron_cmd = 0.063 + 1.302*ref_p + 0.023*beta_deg - 0.042*ay
    aileron_cmd = (-73.594 + 2899.711*ref_p + 42.936*beta_deg + 3.394*airspeed_mps - 80.052*ay) / qbar
    sum_p += (ref_p - p) * 0.02
    if sum_p < -1: sum_p = -1
    if sum_p > 1: sum_p = 1
    aileron_cmd += sum_p * 1.0  # integral gain
    print("ref_p: %.2f" % ref_p, "sum_p: %.2f" % sum_p, "ail: %.2f" % aileron_cmd)
    control_flight_node.setFloat("aileron", aileron_cmd)

    if abs(inceptor_node.getFloat("elevator")) < 0.03:
        if not pitch_cmd_neutral:
            print("set pitch neutral")
            ref_theta = theta_deg
            pitch_cmd_neutral = True
    else:
        pitch_cmd_neutral = False
    if pitch_cmd_neutral:
        ref_q = (ref_theta - theta_deg) * 0.05
        # base_ref_q = ref_q
    else:
        # ref_q = base_ref_q - inceptor_node.getFloat("elevator") * 20 * d2r
        ref_q = -inceptor_node.getFloat("elevator") * 20 * d2r
    # elevator_cmd = (-199.150 - 1574.264*ref_q) / qbar
    # elevator_cmd = (-186.930 - 1611.536*ref_q - 40.039*abs(ay)) / qbar
    # elevator_cmd = (-215.963 - 1599.468*ref_q - 38.223*abs(ay) + 46.013*throttle_cmd) / qbar
    # elevator_cmd = (-332.159 - 1352.235*ref_q - 36.577*abs(ay) + 47.427*throttle_cmd - 11.859*gbody_z) / qbar
    elevator_cmd = (-292.128 - 1304.394*ref_q - 36.907*abs(ay) + 47.111*throttle_cmd - 10.242*gbody_z + 2.421*az) / qbar
    sum_q += (ref_q - q) * 0.02
    if sum_q < -1: sum_q = -1
    if sum_q > 1: sum_q = 1
    elevator_cmd -= sum_q * 1.0  # integral gain
    # print("ref_q: %.2f" % ref_q, "sum_q: %.2f" % sum_q, "ele: %.2f" % elevator_cmd)
    control_flight_node.setFloat("elevator", elevator_cmd)
    control_flight_node.setFloat("rudder", inceptor_node.getFloat("rudder"))
    control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
    control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

    if abs(inceptor_node.getFloat("rudder")) < 0.01:
        if not yaw_cmd_neutral:
            print("set yaw neutral")
            yaw_cmd_neutral = True
    else:
        yaw_cmd_neutral = False
    if yaw_cmd_neutral:
        ref_beta = 0.0
    else:
        ref_beta = -inceptor_node.getFloat("rudder") * 20
    # rudder_cmd = (-10.483 - 85.506*ref_beta) / qbar
    # rudder_cmd = (-58.646 - 79.930*ref_beta + 119.142*gbody_x) / qbar
    # rudder_cmd = (-61.279 - 82.352*ref_beta + 115.594*gbody_x + 921.169*r) / qbar
    r = 0
    rudder_cmd = (-51.718 - 103.900*ref_beta + 100.800*gbody_x + 1005.478*r - 105.357*ay) / qbar
    sum_beta -= (ref_beta - beta_deg) * 0.0001
    if sum_beta < -1: sum_beta = -1
    if sum_beta > 1: sum_beta = 1
    rudder_cmd += sum_beta * 0.1  # integral gain
    print("ref_beta: %.2f" % ref_beta, "sum_beta: %.2f" % sum_beta, "rud: %.2f" % rudder_cmd)
    control_flight_node.setFloat("rudder", rudder_cmd)

def update():
    joystick.update()
    nota_fcs()
    sim.RunSteps(4, updateWind=True)
    sim.PublishProps()

    fgfs.send_to_fgfs()
    # pvi.update(state_mgr, 0, 0, 0, 0)
    # xp.update(state_mgr)

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
