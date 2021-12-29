# rc-sim

Simple flight dynamics model creation and simulation.  Based on a
novel data-driven approach which fits (learns) the flight
characteristics of specific aircraft from their flight data log.  This
system builds a working flight simulator model of your (or any)
specific aircraft directly from the data log.

## Brief description of the method

Assume you have conducted a real UAV flight and logged the data.  Back
on the ground, you scan the recorded log file and assemble a small set
of measurable (or estimatable) states at each time step into a vector.
Now collect all these individual state vectors for every time step
except the last step: (i = 0 to n-1) into a giant matrix called **X**.
Next do the same thing with the same state vectors except drop the
first vector and include the last one (i = 1 to n).  Call this matrix
**Y**.  Now **Y** contains the same state data as **X**, just shifted
over by one position in time.  **X** and **Y** have the same
dimensions.

Now form a simple matrix algebra equation **Y = A * X**

If we can find a matrix **A** that best satisfies this equation in a
least squares sense, then we have a matrix that will take *all* the
current states and compute an estimate of all the next states.  Cool,
huh?

We know the values of the matrices **X** and **Y**, so we can quickly
solve for the **A** matrix.  We can't directly invert **X** because it
is not square, but there are a number of ways to proceed. I chose to
decompose the **X** matrix using a singular value decomposition (SVD),
then apply a few basic matrix algebra steps to isolate **A** on one
side of the equation ... and that is it -- all done using a couple
lines of numpy code.

For fluids people, the initial construction of this process is
identical to Dynamic Mode Decomposition, except here we want to solve
for the actual **A** matrix.  DMD uses additional simplifications
because it is primarily concerned with finding the eigenvalues and
eigenvectors of the **A** matrix.

## States

What states do we care about?  This is actually simpler than you might
expect.

### Independent states

* Throttle command -- we use sqrt(throttle) to crudely approximate a
  thrust curve.
* Aileron, Elevator, Rudder commands (approximation of the control
  surface positions) -- these are multiplied by qbar so their
  effectiveness is scaled with airspeed.
* The gravity vector rotated into the body frame of reference.  This
  requires an estimate of the roll and pitch Euler angles which we
  typically have on any small uav running an EKF.  The body frame
  gravity vector is used instead of roll and pitch angles.

### Dependent states

* X, Y, and Z velocities in the body frame of reference.  These can be
  computed from the (wind corrected) NED velocity rotated into the
  body frame.  These are also multiplied by qbar so that angle of
  attack, side-slip, and drag effects scale properly with airspeed.
* Rotational rates (p, q, r)

That's it, that is all it is!  Well, we additionally include abs(rudder)
and abs(aileron) to consider their non-directional effects.  In the
current form, the **A** matrix dimensions are 16x16.  With this matrix
we can predict any next state from any current state.

## Simulation

Once the **A** matrix is computed, this forms the heart of the flight
simulator.

The simulation has a simple update loop:

* The independent variables can be attached to external inputs (like a
  physical joystick) and used to update the current state vector.  The
  simulation keeps a running estimate of aircraft body estimate, so
  gravity can always be rotated into the current body frame.

* The dependent variables start out with some initial estimate (like
  zero) and are updated to the next state estimate produced by
  multiplying the previous state by the **A** matrix.  These estimates
  of the dependent variables get carried forward from step to step.

* With each new state estimation, the aircraft body orientation can be
  updated from the new p, q, r estimates.  Then the updated aircraft
  body frame velocities can be rotated into the NED frame to get NED
  velocities.  And finally the NED position can be updated.  There are
  some additional bookkeeping details, but this is essentially all
  that is needed to create a realistic flight simulation from the best
  fit model (aka the **A** matrix.)

## Limitations / Disclaimers

This system is not a perfect or complete answer to creating flight
simulations from real flight data.  There are many limitations and
disclaimers, here are just a few:

* Only works for in-flight state (no ground reactions modeled.)
* Common variables (like air density, aircraft mass, aircraft cg) get
  cooked directly into the model and cannot be varied in the
  simulation.
* Simulation results are most valid within the range of flight
  conditions that were recorded.  Extrapolating beyond these ranges
  may or may not produce some strange or exaggerated effects.
  Definitely do not depend on useful results outside the range of
  state data collected in the flight.
* The fit (approximation) is linear, so when possible the input
  parameters are scaled for a better linear fit (i.e. multiplying
  certain parameters by qbar.)  But the fit is still linear.
* Traditional aerodynamic coefficients are bypassed. New rotational
  rates and body frame velocities are updated estimated/updated every
  frame.  This creates a model the closely matches the performance of
  the original flight data (good) but it cannot be decomposed into
  coefficient parts as with traditional methods.

## Other use cases

The **A** matrix is built for a specific aircraft at a specific
configuration.  This may not be great for full scale aircraft, but it
is common to fly small battery powered UAV's with a fixed
configuration and payload.

* Integrity monitor: Once the model is created, the system is light
  weight enough to run on a typical small UAV flight controller.  the
  current state can be used to predict the expected next state.  That
  can be compared to the true next state and outliers can be flagged
  when they exceed some error threshold.  In addition the system has
  some simple knowledge of which input states are used to predict the
  dependent states, so it may be possible to further isolate an error
  condition to a specific set of inputs.

* The system will predict X, Y, Z velocities in the body frame of
  reference.  The norm() of this velocity vector is the true airspeed.
  Thus, once the model is created, it can be used to estimate/predict
  a synthetic airspeed without needing an actual airspeed sensors.

* For future work, if wind vector estimates are included in the state
  list, it may be possible to use the system to estimate/predict wind
  in all 3 axes.  This could be an alternative to traditional
  in-flight wind speed estimators.  Or this could be used as a way to
  estimate the vertical velocity of the air column for soaring
  applications.  (Note: not yet attempted, just hypothetical for now.)

* This is a fairly generic approach to estimating next state from
  current state.  It could be used for non-flight related predictions.
  Or additional states could be added to the state vector to include
  more effects for a better performance fit.

* Currently this system is only tested for fixed wing models with an
  airspeed sensor.  I hope to think through what would be involved in
  adapting this to fitting multirotor models.