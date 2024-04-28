#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

from apscheduler.schedulers.background import BackgroundScheduler   # pip install APScheduler (dnf install python3-APScheduler)
import argparse
import os
import time

from lib.props import control_engine_node, control_flight_node, inceptor_node, vel_node
from FCS.FCS_pbeta import FCS_pbeta
from FCS.FCS_q import FCS_q
from FCS.IsFlying import IsFlying
from sim.jsbsim import JSBSimWrap
from sim.joystick import Joystick
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
# pathJSB = os.path.join("/home/clolson/Projects/SVO_Simulator/simulation-python-jsbsim", "JSBSim")
# pathJSB = os.path.join("/Users/Cirrus/Projects/SVO_Simulator/simulation-python-jsbsim", "JSBSim")
pathJSB = os.path.join("/home/curt/Sync", "JSBSim")
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

fcs_lat = FCS_pbeta()
fcs_lon = FCS_q()
# fcs = FCS_pr_q()
is_flying = IsFlying(on_ground_for_sure_mps=30, flying_for_sure_mps=40)

def update():
    joystick.update()

    # this section should go into a top level FCS() class
    flying_confidence = is_flying.get_flying_confidence(vel_node.getFloat("vc_mps"))
    fcs_lat.update(flying_confidence)
    fcs_lon.update(flying_confidence)
    print("integrators: %.2f %.2f %.2f" % (fcs_lat.aileron_int, fcs_lon.elevator_int, fcs_lat.rudder_int))  # move outside

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
