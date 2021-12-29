# rc-sim

Simple flight dynamics model creation and simulation.  Based on a
novel data-driven approach which fits (learns) the flight characters
of specific aircraft from their flight data log.

## Brief description of the method

Assume you have conducted a real UAV flight and logged the data.  Back
on the ground, you scan the recorded log file and assemble a small set
of measurable (or estimatable) states at each time step into a vector.
Now collect all these individual state vectors for every time step in
the data log (except the last step: i = 0 to n-1) into a giant matrix
called **X**.  Next do the same thing with the same state vectors
except drop the first vector and include the last one (i = 1 to n).
Call this matrix **Y**.  Now **Y** contains the same state data as
**X**, just shifted over by one position in time.  **X** and **Y**
have the same dimensions.

Now form a simple matrix algebra equation **Y = A * X**

If we can find a matrix **A** that best satisfies this equation in a
least squares sense, then we have a matrix that will take *all* the
current states and compute an estimate of all the next states.  Cool,
huh?

We know the values of the mattrices **X** and **Y**, so we can quickly
solve for the **A** matrix.  We can't directly invert **X** because it
is not square, but there are a number of ways to proceed. I chose to
decompose the X matrix using a singular value decomposition (SVD),
then apply a few basic matix algebra steps to isolate **A** on one
side of the equation ... and done using a couple lines of numpy code.

For fluids people, the initial construction of this process is
identical to Dynamic Mode Decomposition, except here we want to solve
for the actual **A** matrix.  DMD uses additional simplifications
because it is only concerned with finding the eigenvalues and
eigenvectors of the **A** matrix.

## States

What states do we care about?  This is actually simpler than you might
expect.

### Independent states

* Throttle command -- we use sqrt(throttle) to crudely approximate a
  thrust curve.
* Aileron, Elevator, Rudder commands (flight controls) -- these are
  multipled by qbar so their effectiveness is scaled with airspeed.
* The gravity vector rotated into the body frame of reference.  This
  requires an estimate of the roll and pitch euler angles which we
  typically have on any small uav running an EKF.

### Dependent states

* X, Y, and Z velocities in the body frame of reference.  These can be
  computed from the (wind corrected) NED velocity rotated into the
  body frame.  These are also multiplied by qbar so that alpha, beta,
  and drag effects scale properly with airspeed.
* Rotational rates (p, q, r)

That's it, that's all it is!  Well, we additionaly add abs(rudder) and
abs(aileron) to consider their non-directional effects.

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