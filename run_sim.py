#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
import numpy as np
import time
from apscheduler.schedulers.background import BackgroundScheduler # pip install APScheduler (dnf install python3-APScheduler)

from lib.joystick import Joystick
from lib.simulator import Simulator
from visuals.fgfs import fgfs
from visuals.xp.xp import XPlane

# command line arguments
parser = argparse.ArgumentParser(description="run the simulation")
parser.add_argument("model", help="flight model")
parser.add_argument('--realtime', action='store_true', help='run sim in realtime')
parser.add_argument('--no-trim', action='store_true', help="don't trim")
parser.add_argument('--rudder-expo', type=float, default=1)
args = parser.parse_args()

run_time = 600

joystick = Joystick()
sim = Simulator()
sim.load(args.model)
sim.reset()
if not args.no_trim:
    sim.trim(20)

xp = XPlane()

def expo(x, y):
    print(x, y)
    result = x**y
    if np.sign(result) != np.sign(x):
        result *= -1
    return result

def update():
    joystick.update()
    sim.state_mgr.set_throttle(joystick.throttle)
    sim.state_mgr.set_flight_surfaces( joystick.aileron,
                                       joystick.elevator,
                                       expo(joystick.rudder, args.rudder_expo)
                                      )
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
