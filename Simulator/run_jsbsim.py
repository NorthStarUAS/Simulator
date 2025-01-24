#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

from apscheduler.schedulers.background import BackgroundScheduler   # pip install APScheduler (dnf install python3-APScheduler)
import argparse
from pathlib import Path
import time

from comms.HIL_nsLink import HIL
from FCS.fcs_mgr import FCSMgr
from sim.jsbsim import JSBSimWrap
from sim.joystick import Joystick
from visuals.fgfs import fgfs
from visuals.display import Display
from visuals.xp.xp import XPlane

# command line arguments
parser = argparse.ArgumentParser(description="run the simulation")
parser.add_argument("model", help="flight model")
parser.add_argument('--realtime', action='store_true', help='run sim in realtime')
parser.add_argument('--no-trim', action='store_true', help="don't trim")
args = parser.parse_args()

# high level sim will run at a 50 hz schedule
fdm_steps_per_frame = 4
dt = 1 / (fdm_steps_per_frame * 50)

joystick = Joystick()
display = Display()
xp = XPlane()
hil = HIL()

home = Path.home()

if False:
    model = 'Rascal110'
    pathJSB = Path("./models_jsbsim")

if True:
    model = 'SR22T'
    #pathJSB = home / "Projects/ADD_Simulator/simulation-python-jsbsim/JSBSim"
    pathJSB = home / "Sync/JSBSim"

print("JSBSim path:", pathJSB)

sim = JSBSimWrap(model, pathJSB.as_posix(), dt=dt)
sim.SetupICprops()

if not args.no_trim: # fixme
    # setting this property invokes the JSBSim trim routine
    sim.fdm['simulation/do_simple_trim'] = 0  # In-air trim
    # sim.fdm['simulation/do_simple_trim'] = 2  # Ground trim

sim.SetTurb(turbSeverity=1, vWind20_mps=2, vWindHeading_deg=45) # Trim with wind, no turbulence

fcs = FCSMgr()

def update():
    joystick.update()
    hil.read()
    fcs.update()
    # hil.read()
    sim.RunSteps(fdm_steps_per_frame, updateWind=True)
    sim.PublishProps()
    hil.write()
    fgfs.send_to_fgfs()
    display.update()
    xp.update()

if args.realtime:
    sched = BackgroundScheduler()
    sched.add_job(update, 'interval', seconds=1/50)
    sched.start()
    while True:
        time.sleep(1)
    sched.shutdown()
else:
    while sim.time <= run_time:
        sim.update()

sim.plot()
