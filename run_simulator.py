#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import argparse
from sim import Simulator

# command line arguments
parser = argparse.ArgumentParser(description="simulation front end")
parser.add_argument("model", help="flight model")
args = parser.parse_args()

sim = Simulator()
sim.load(args.model)
sim.reset()

# run the sim with fixed inputs for a number of seconds.
while sim.time < 120:
    sim.update()

sim.plot()
