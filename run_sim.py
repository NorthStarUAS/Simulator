#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
import time
from apscheduler.schedulers.background import BackgroundScheduler # dnf install python3-APScheduler

import fgfs
import joystick
from simulator import Simulator

# command line arguments
parser = argparse.ArgumentParser(description="simulation front end")
parser.add_argument("model", help="flight model")
parser.add_argument('--realtime', action='store_true', help='run sim in realtime')
parser.add_argument('--no-trim', action='store_true', help="don't trim")
args = parser.parse_args()

run_time = 1000

sim = Simulator()
sim.load(args.model)
sim.reset()
if not args.no_trim:
    sim.trim(20)

def update():
    throttle, aileron, elevator, rudder = joystick.update()
    sim.state_mgr.set_throttle(throttle)
    sim.state_mgr.set_flight_surfaces(aileron, elevator, rudder)
    sim.update()
    fgfs.send_to_fgfs(sim)

if args.realtime:
    sched = BackgroundScheduler()
    sched.add_job(update, 'interval', seconds=sim.dt)
    sched.start()
    time.sleep(run_time)
    sched.shutdown()
else:
    while sim.time <= run_time:
        sim.update()

sim.plot()
