#!/usr/bin/env python3

from sim import Simulator

sim = Simulator()
sim.load("skywalker_model.json")
sim.reset()

while sim.time < 60:
    sim.update()

sim.plot()
