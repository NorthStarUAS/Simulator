#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

from apscheduler.schedulers.background import BackgroundScheduler # pip install APScheduler (dnf install python3-APScheduler)
import argparse
import json
import numpy as np
import time

from lib.joystick import Joystick
from lib.simulator import Simulator
from visuals.fgfs import fgfs
from visuals.xp.xp import XPlane

# command line arguments
parser = argparse.ArgumentParser(description="run the simulation")
parser.add_argument("model", help="flight model")
parser.add_argument('--realtime', action='store_true', help='run sim in realtime')
parser.add_argument('--no-trim', action='store_true', help="don't trim")
args = parser.parse_args()

run_time = 600

joystick = Joystick()
xp = XPlane()

# load in the beast!
f = open(args.model, "r")
model = json.load(f)
print(model)
f.close()

rows = model["rows"]
cols = model["cols"]
dt = model["dt"]
for condition in model["conditions"]:
    sim = Simulator()
    condition["sim"] = sim
    A = np.array(condition["A"]).reshape(rows, cols)
    sim.setup(dt, A, condition["parameters"])
    sim.reset()

if not args.no_trim: # fixme
    sim.trim(20)

def update():
    global model
    joystick.update()
    sim = None
    for i, condition in enumerate(model["conditions"]):
        print(i, condition["condition"])
        if "flaps" in condition["condition"] and abs(joystick.flaps - condition["condition"]["flaps"]) < 0.1:
            sim = condition["sim"]
            break
    sim.state_mgr.set_throttle(joystick.throttle)
    sim.state_mgr.set_flight_surfaces(joystick.aileron, joystick.elevator, joystick.rudder)
    sim.update()
    fgfs.send_to_fgfs(sim)
    xp.update(sim)

if args.realtime:
    sched = BackgroundScheduler()
    sched.add_job(update, 'interval', seconds=sim.dt)
    sched.start()
    while True:
        time.sleep(run_time)
    sched.shutdown()
else:
    while sim.time <= run_time:
        sim.update()

sim.plot()
