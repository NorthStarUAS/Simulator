#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
import time
from apscheduler.schedulers.background import BackgroundScheduler # dnf install python3-APScheduler


from simulator import Simulator

# command line arguments
parser = argparse.ArgumentParser(description="simulation front end")
parser.add_argument("model", help="flight model")
parser.add_argument('--realtime', action='store_true', help='run sim in realtime')
args = parser.parse_args()

run_time = 120

sim = Simulator()
sim.load(args.model)
sim.reset()
sim.trim(20)

def update():
    sim.update()

if args.realtime:
    sched = BackgroundScheduler()
    sched.add_job(update, 'interval', seconds=sim.dt)
    sched.start()
    time.sleep(run_time)
    sched.shutdown()
else:
    while sim.time <= run_time:
        update()

sim.plot()
