#!/usr/bin/env python3

"""run_simulator

Front end to the simulation module

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

from sim import Simulator

sim = Simulator()
sim.load("skywalker_model.json")
sim.reset()

while sim.time < 120:
    sim.update()

sim.plot()
