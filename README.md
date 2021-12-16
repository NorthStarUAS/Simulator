# rc-sim

Flight dynamics model creation and simulation--based on a novel
data-driven approach.

## Overview

Imagine conducting a real UAV flight and logging the data.  Back on
the ground, take the recorded log file and assemble the set of
measurable and relevant states at each time step into a vector. Now
collect all these individual state vectors for every time step (except
the last step: i = 0 to n-1) into a giant matrix called **X**.  Next
do something similar with the same state set of state vectors.
Assemble all but the first state (i = 1 to n) into a giant matrix
called **Y**.  Now **Y** contains the same state data as **X**, just
shifted over by one position.  **X** and **Y** have the same
dimensions.

Now form a simple matrix algebra equation **Y = A * X**

If we can find a matrix **A** that best satisfies this equation in a
least squares sense, then we have a matrix that will take any current
state and predict the next state.  Cool, huh?

We can quickly solve for the **A** matrix purely by assembling flight
log data into the matrices **X** and **Y**.  We don't need to know
anything about the aircraft metrics or first order principles or aero
coefficients, we just need the measurable flight data.

For fluids people, the construction of this process is equivalent to
Dynamic Mode Decomposition, except here we want to solve for the
actual **A** matrix.  DMD uses additional simplifications because it
is only concerned with finding the eigenvalues and eigenvectors of the
**A** matrix.

## States

What states do we care about?  This is actually simpler than you might
expect.

### Independent states

* Throttle command
* Aileron, Elevator, Rudder commands (flight controls)
* Roll and Pitch angles (phi, theta)

### Dependent states

* Airspeed
* Vertical velocity (experimental)
* Body accelerations (ax, ay, az with gravity removed.)
* Rotational rates (p, q, r)

That's it, that's all it is!

## Simulation

Once the **A** matrix is computed, this can form the heart of a
simulation.

# Limitations

This system is not a perfect complete answer to creating flight
simulations from real flight data.  There are many limitations and
disclaimers, here are a few:

* Only works for in-flight state (no ground reactions modeled.)
* Common variables (like air density, aircraft mass, aircraft cg) get
  cooked directly into the model and cannot be varied in the
  simulation.
* Simulation results are most valid within the range of flight
  conditions that were recorded.  Extrapolating beyond these ranges
  may or may not produce useful results, definitely do not depend on
  useful results outside the range of state data collected in the
  flight.
* When things blow up for whichever reason, it can be quite spectacular!